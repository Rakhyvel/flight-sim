use apricot::input::Input;

/// What the pilot is commanding
pub struct Controls {
    // Flight controls
    pub pitch: f32,    // -1 to 1
    pub roll: f32,     // -1 to 1
    pub yaw: f32,      // -1 to 1
    pub throttle: f32, // 0 to 1

    // Component controls
    pub brakes: f32,
    /// Flap switch this frame, +1 a notch down, -1 a notch up, 0 left alone
    pub flap_step: i32,

    // Internal target states for smoothing
    target_pitch: f32,
    target_roll: f32,
    target_yaw: f32,
    target_brakes: f32,
}

impl Controls {
    pub fn new() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            throttle: 0.0,

            brakes: 0.0,
            flap_step: 0,

            target_pitch: 0.0,
            target_roll: 0.0,
            target_yaw: 0.0,
            target_brakes: 0.0,
        }
    }

    pub fn update_input(&mut self, input: &Input, dt: f32) {
        self.target_roll = 0.0;
        self.target_pitch = 0.0;
        self.target_yaw = 0.0;
        self.target_brakes = 0.0;
        self.flap_step = 0;

        if input.key_down(sdl2::keyboard::Scancode::LShift) {
            self.throttle += 0.01;
        }
        if input.key_down(sdl2::keyboard::Scancode::LCtrl) {
            self.throttle -= 0.01;
        }
        self.throttle = self.throttle.clamp(0.0, 1.0);

        // TODO: This is kinda a mess but I feel like this is the only way?
        if input.key_down(sdl2::keyboard::Scancode::W) {
            self.target_pitch -= 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::S) {
            self.target_pitch += 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::A) {
            self.target_yaw += 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::D) {
            self.target_yaw -= 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::Q) {
            self.target_roll -= 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::E) {
            self.target_roll += 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::B) {
            self.target_brakes = 1.0;
        }
        if input.key_pressed(sdl2::keyboard::Scancode::F) {
            self.flap_step += 1;
        }
        if input.key_pressed(sdl2::keyboard::Scancode::V) {
            self.flap_step -= 1;
        }

        // Smooth the effects
        let smoothing = 5.0 * dt;
        self.pitch += (self.target_pitch - self.pitch) * smoothing.min(1.0);
        self.roll += (self.target_roll - self.roll) * smoothing.min(1.0);
        self.yaw += (self.target_yaw - self.yaw) * smoothing.min(1.0);

        self.brakes += (self.target_brakes - self.brakes) * smoothing.min(1.0);

        if let Some(js) = input.joystick() {
            let stick_roll = js.axis(0);
            let stick_pitch = js.axis(1);
            let stick_yaw = js.axis(2);

            self.pitch = response_curve(stick_pitch, 0.5);
            self.roll = response_curve(stick_roll, 0.5);
            self.yaw = -response_curve(stick_yaw, 0.5);

            self.throttle = (1.0 - js.axis(3)) * 0.5;
        }
    }
}

fn response_curve(x: f32, cubic: f32) -> f32 {
    let x3 = x * x * x;
    x * (1.0 - cubic) + x3 * cubic
}
