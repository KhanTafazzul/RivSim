//! Parallel Headless Batch Generator using Rayon
//! Spawns multiple parallel Rapier2D physics worlds across CPU cores.
//! Generates 1M-5M rows of return-to-home RL training data at maximum CPU speed.

use std::io::Write;
use std::path::Path;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Instant;
use rayon::prelude::*;

use crate::dataset::{create_csv_dataset, run_episode, EpisodeConfig};

pub struct HeadlessBatchRunner;

impl HeadlessBatchRunner {
    /// Run parallel headless episodes across CPU cores until target_rows is satisfied
    pub fn run_batch<P: AsRef<Path>>(
        num_episodes: u32,
        target_rows: u32,
        output_csv_path: P,
        enable_exploration: bool,
    ) -> (u32, u32, f64) {
        let start_time = Instant::now();
        let path = output_csv_path.as_ref();

        println!(
            "[RivSim Headless] Initializing Rayon 4-core batch simulation for {} episodes (target {} rows)...",
            num_episodes, target_rows
        );

        let writer = match create_csv_dataset(path) {
            Ok(w) => Arc::new(std::sync::Mutex::new(w)),
            Err(e) => {
                eprintln!("[RivSim Headless] Failed to create dataset file {}: {}", path.display(), e);
                return (0, 0, 0.0);
            }
        };

        let total_rows_counter = Arc::new(AtomicU32::new(0));
        let successful_returns_counter = Arc::new(AtomicU32::new(0));

        let episodes: Vec<u32> = (0..num_episodes).collect();

        episodes.par_iter().for_each(|&ep_id| {
            // Check if row target already satisfied
            if total_rows_counter.load(Ordering::Relaxed) >= target_rows && target_rows > 0 {
                return;
            }

            let cfg = EpisodeConfig::random(ep_id, enable_exploration);

            // Thread-local memory buffer to minimize mutex contention
            let mut local_buf = Vec::with_capacity(32 * 1024);
            let (rows, reached_home, _, _) = run_episode(&cfg, &mut local_buf, 50.0);

            // Flush local buffer to shared file writer
            if let Ok(mut locked_writer) = writer.lock() {
                let _ = locked_writer.write_all(&local_buf);
            }

            total_rows_counter.fetch_add(rows, Ordering::Relaxed);
            if reached_home {
                successful_returns_counter.fetch_add(1, Ordering::Relaxed);
            }
        });

        // Ensure everything is flushed
        if let Ok(mut locked_writer) = writer.lock() {
            let _ = locked_writer.flush();
        }

        let elapsed_sec = start_time.elapsed().as_secs_f64();
        let total_rows = total_rows_counter.load(Ordering::Relaxed);
        let success_count = successful_returns_counter.load(Ordering::Relaxed);

        let rows_per_sec = if elapsed_sec > 0.0 {
            total_rows as f64 / elapsed_sec
        } else {
            0.0
        };

        println!(
            "[RivSim Headless] Completed {} rows ({} episodes, {} successful returns) in {:.2}s ({:.0} steps/sec)",
            total_rows, num_episodes, success_count, elapsed_sec, rows_per_sec
        );

        (total_rows, success_count, elapsed_sec)
    }
}
