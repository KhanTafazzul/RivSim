//! Dynamic C++ ESP32 Firmware Compiler & libloading Host
//! Automatically compiles uploaded .cpp/.ino sketches using cl.exe / g++ / clang++
//! and hot-loads them into the running simulation.

use std::path::{Path, PathBuf};
use std::process::Command;
use libloading::{Library, Symbol};
use crate::firmware_host::bridge::HalBridgeState;

type FnInit = unsafe extern "C" fn(*mut HalBridgeState);
type FnSetup = unsafe extern "C" fn();
type FnLoop = unsafe extern "C" fn();

pub struct DynamicFirmwareHost {
    _library: Option<Library>,
    fn_loop: Option<FnLoop>,
    pub bridge_state: Box<HalBridgeState>,
    pub is_loaded: bool,
    pub loaded_source: Option<String>,
    pub last_compiler_output: String,
}

impl DynamicFirmwareHost {
    pub fn new() -> Self {
        Self {
            _library: None,
            fn_loop: None,
            bridge_state: Box::new(HalBridgeState::new()),
            is_loaded: false,
            loaded_source: None,
            last_compiler_output: String::new(),
        }
    }

    /// Compile a C++ sketch file and hot-load the resulting dynamic library
    pub fn compile_and_load<P: AsRef<Path>>(&mut self, source_path: P) -> Result<(), String> {
        let src_path = source_path.as_ref();
        if !src_path.exists() {
            return Err(format!("Source file does not exist: {}", src_path.display()));
        }

        let out_dir = std::env::temp_dir().join("rivsim_firmware");
        let _ = std::fs::create_dir_all(&out_dir);

        #[cfg(target_os = "windows")]
        let dll_path = out_dir.join("rivsim_firmware.dll");
        #[cfg(not(target_os = "windows"))]
        let dll_path = out_dir.join("librivsim_firmware.so");

        // Attempt compilation using available toolchain
        let compile_result = self.invoke_compiler(src_path, &dll_path);
        match compile_result {
            Ok(output) => {
                self.last_compiler_output = output;
            }
            Err(err) => {
                self.last_compiler_output = err.clone();
                return Err(err);
            }
        }

        // Load compiled shared library via libloading
        unsafe {
            let lib = Library::new(&dll_path).map_err(|e| format!("Failed to link dynamic library: {}", e))?;
            
            // Resolve symbols
            let init: Symbol<FnInit> = lib.get(b"firmware_init")
                .map_err(|e| format!("Missing extern \"C\" void firmware_init(HalBridgeState*): {}", e))?;
            let setup: Symbol<FnSetup> = lib.get(b"firmware_setup")
                .map_err(|e| format!("Missing extern \"C\" void firmware_setup(): {}", e))?;
            let loop_fn: Symbol<FnLoop> = lib.get(b"firmware_loop")
                .map_err(|e| format!("Missing extern \"C\" void firmware_loop(): {}", e))?;

            // Initialize bridge pointer and run setup()
            let raw_bridge = &mut *self.bridge_state as *mut HalBridgeState;
            init(raw_bridge);
            setup();

            self.fn_loop = Some(*loop_fn);
            self._library = Some(lib);
            self.is_loaded = true;
            self.loaded_source = Some(src_path.to_string_lossy().to_string());
        }

        Ok(())
    }

    /// Step the loaded firmware loop function
    pub fn step(&mut self) {
        if let Some(loop_fn) = self.fn_loop {
            unsafe {
                loop_fn();
            }
        }
    }

    /// Internal compiler detection and invocation
    fn invoke_compiler(&self, src: &Path, out_dll: &Path) -> Result<String, String> {
        let mock_hal_dir = std::env::current_dir()
            .unwrap_or_else(|_| PathBuf::from("."))
            .join("mock_hal");

        let include_arg = format!("-I{}", mock_hal_dir.display());

        // 1. Check for g++
        let gxx_check = Command::new("g++").arg("--version").output();
        if let Ok(_) = gxx_check {
            let output = Command::new("g++")
                .arg("-shared")
                .arg("-fPIC")
                .arg("-O2")
                .arg(&include_arg)
                .arg(src)
                .arg("-o")
                .arg(out_dll)
                .output()
                .map_err(|e| format!("g++ execution failed: {}", e))?;

            if output.status.success() {
                return Ok(String::from_utf8_lossy(&output.stdout).to_string());
            } else {
                return Err(String::from_utf8_lossy(&output.stderr).to_string());
            }
        }

        // 2. Check for clang++
        let clang_check = Command::new("clang++").arg("--version").output();
        if let Ok(_) = clang_check {
            let output = Command::new("clang++")
                .arg("-shared")
                .arg("-fPIC")
                .arg("-O2")
                .arg(&include_arg)
                .arg(src)
                .arg("-o")
                .arg(out_dll)
                .output()
                .map_err(|e| format!("clang++ execution failed: {}", e))?;

            if output.status.success() {
                return Ok(String::from_utf8_lossy(&output.stdout).to_string());
            } else {
                return Err(String::from_utf8_lossy(&output.stderr).to_string());
            }
        }

        // 3. Check for MSVC cl.exe
        #[cfg(target_os = "windows")]
        {
            let cl_check = Command::new("cl.exe").arg("/?").output();
            if let Ok(_) = cl_check {
                let include_msvc = format!("/I{}", mock_hal_dir.display());
                let fe_arg = format!("/Fe:{}", out_dll.display());
                let output = Command::new("cl.exe")
                    .arg("/LD")
                    .arg("/O2")
                    .arg("/EHsc")
                    .arg(&include_msvc)
                    .arg(src)
                    .arg(&fe_arg)
                    .output()
                    .map_err(|e| format!("cl.exe execution failed: {}", e))?;

                if output.status.success() {
                    return Ok(String::from_utf8_lossy(&output.stdout).to_string());
                } else {
                    return Err(String::from_utf8_lossy(&output.stderr).to_string());
                }
            }
        }

        Err("No suitable C++ compiler (g++, clang++, or cl.exe) found on PATH. Falling back to native Rust firmware controller.".to_string())
    }
}

