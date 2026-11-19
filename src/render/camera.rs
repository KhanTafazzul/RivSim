//! 2D Viewport Camera with Smooth Following, Panning, and Zooming

use macroquad::prelude::*;

#[derive(Debug, Clone)]
pub struct SimulationCamera {
    pub target: Vec2,
    pub zoom: f32,
    pub follow_boat: bool,
    pub drag_start: Option<Vec2>,
}

impl SimulationCamera {
    pub fn new() -> Self {
        Self {
            target: vec2(0.0, 0.0),
            zoom: 24.0, // Pixels per meter
            follow_boat: true,
            drag_start: None,
        }
    }

    /// World meter coordinate to screen pixel coordinate
    pub fn world_to_screen(&self, world_pos: Vec2) -> Vec2 {
        let screen_center = vec2(screen_width() * 0.5, screen_height() * 0.5);
        let delta = world_pos - self.target;
        // Invert Y so positive Y is up in world space
        vec2(
            screen_center.x + delta.x * self.zoom,
            screen_center.y - delta.y * self.zoom,
        )
    }

    /// Screen pixel coordinate to world meter coordinate
    pub fn screen_to_world(&self, screen_pos: Vec2) -> Vec2 {
        let screen_center = vec2(screen_width() * 0.5, screen_height() * 0.5);
        let delta = screen_pos - screen_center;
        vec2(
            self.target.x + delta.x / self.zoom,
            self.target.y - delta.y / self.zoom,
        )
    }

    /// Update camera controls: mouse drag pan, scroll zoom, and follow boat mode
    pub fn update(&mut self, boat_pos: Vec2) {
        if self.follow_boat {
            self.target = self.target.lerp(boat_pos, 0.08);
        }

        // Toggle follow mode with 'F' key
        if is_key_pressed(KeyCode::F) {
            self.follow_boat = !self.follow_boat;
        }

        // Zoom with mouse wheel
        let wheel_y = mouse_wheel().1;
        if wheel_y > 0.0 {
            self.zoom = (self.zoom * 1.12).min(80.0);
        } else if wheel_y < 0.0 {
            self.zoom = (self.zoom / 1.12).max(6.0);
        }

        // Pan with right mouse drag
        if is_mouse_button_pressed(MouseButton::Right) {
            self.drag_start = Some(mouse_position().into());
        }
        if is_mouse_button_down(MouseButton::Right) {
            if let Some(start) = self.drag_start {
                let current: Vec2 = mouse_position().into();
                let delta = current - start;
                self.target.x -= delta.x / self.zoom;
                self.target.y += delta.y / self.zoom;
                self.drag_start = Some(current);
                self.follow_boat = false;
            }
        } else {
            self.drag_start = None;
        }
    }
}

