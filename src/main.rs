use std::time::Instant;

// 1. 定義物理環境
struct PhysicalSystem {
    mass: f64,
    damping: f64,
    spring_k: f64,
    position: f64,
    velocity: f64,
}

impl PhysicalSystem {
    fn new(m: f64, c: f64, k: f64) -> Self {
        PhysicalSystem {
            mass: m,
            damping: c,
            spring_k: k,
            position: 0.0,
            velocity: 0.0,
        }
    }

    // 數值積分：根據推力 F 和時間間隔 dt 更新物理狀態
    fn update(&mut self, force: f64, dt: f64) {
        // a = (F - c*v - k*x) / m
        let acceleration =
            (force - self.damping * self.velocity - self.spring_k * self.position) / self.mass;
        self.velocity += acceleration * dt;
        self.position += self.velocity * dt;
    }
}

// 2. 定義 PID 控制器
struct PID {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    prev_error: f64,
}

impl PID {
    fn compute(&mut self, setpoint: f64, measured: f64, dt: f64) -> f64 {
        let error = setpoint - measured;
        self.integral += error * dt;
        let derivative = (error - self.prev_error) / dt;
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        self.prev_error = error;
        output
    }
}

fn main() {
    let mut drone = PhysicalSystem::new(1.0, 0.5, 2.0); // 模擬一個有阻尼的物體
    let mut controller = PID {
        kp: 10.0,
        ki: 2.0,
        kd: 1.0,
        integral: 0.0,
        prev_error: 0.0,
    };

    let target = 10.0; // 目標位置
    let dt = 0.01; // 模擬步長 (10ms)

    println!("Time(s) | Position | Visualization");
    println!("---------------------------------");

    for step in 0..500 {
        let time = step as f64 * dt;
        let force = controller.compute(target, drone.position, dt);
        drone.update(force, dt);

        if step % 10 == 0 {
            // 每 100ms 印一次結果
            let visual_pos = (drone.position * 4.0) as usize; // 縮放比例用於顯示
            println!(
                "{:7.2} | {:8.2} | {:>width$}",
                time,
                drone.position,
                "●",
                width = visual_pos.min(60)
            );
        }
    }
}
