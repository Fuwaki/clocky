use std::{
    iter::once_with,
    sync::{Arc, Mutex, OnceLock},
    thread, vec,
};

use chrono::{Local, Timelike};
use vello::{
    Scene,
    kurbo::{Affine, Circle, Ellipse, Line, RoundedRect, Stroke},
    peniko::Color,
};

mod render;
mod simulation;
static CONTEXT: OnceLock<Mutex<Vec<(f32, f32, f32)>>> = OnceLock::new();
fn render(scene: &mut Scene) {
    CONTEXT
        .get_or_init(|| Mutex::new(vec![]))
        .lock()
        .unwrap()
        .iter()
        .for_each(|(x, y, z)| {
            let circle = Circle::new((*x, *y), *z as f64);
            let circle_fill_color = Color::new([0.2121, 0.5292, 0.6749, 1.]);
            scene.fill(
                vello::peniko::Fill::NonZero,
                Affine::IDENTITY,
                circle_fill_color,
                None,
                &circle,
            );
        });
}
fn get_time() -> (u32, u32, u32, u32) {
    let now = Local::now();
    let (_, hour) = now.hour12();
    let minute = now.minute();
    let second = now.second();
    let millisecond = now.timestamp_subsec_millis(); // 直接获取毫秒部分
    (hour, minute, second, millisecond)
}
fn time_to_clock_angle(hour: u32, minute: u32, second: u32, millisecond: u32) -> (f32, f32, f32) {
    let hour_angle = (hour as f32 % 12.0 + minute as f32 / 60.0 + second as f32 / 3600.0) * 30.0;
    let minute_angle = (minute as f32 + second as f32 / 60.0 + millisecond as f32 / 60000.0) * 6.0;
    let second_angle = (second as f32 + millisecond as f32 / 1000.0) * 6.0;
    (hour_angle, minute_angle, second_angle)
}

fn simu_thread(simu: Arc<Mutex<simulation::Simulation>>) {
    let mut inited = false;
    let mut i=0.05;
    loop {
        let mut simu = simu.lock().unwrap();
        if !inited {
            simu.random_init(7000);
            inited = true;
        }
        let (hour, minute, second, millisecond) = get_time();
        i+=0.001;
        let mut angles =
            time_to_clock_angle(hour, minute, second, millisecond);
            angles.0 = i;  
        simu.set_clock_angle(angles);

        let res = simu.update();
        // println!("{:?}",res);
        let mut guard = CONTEXT.get_or_init(|| Mutex::new(vec![])).lock().unwrap();
        *guard = res;
        drop(simu);
        thread::sleep(std::time::Duration::from_millis(2));
        
    }
}

fn main() -> anyhow::Result<()> {
    let simu = simulation::Simulation::new();
    let simu_for_thread = Arc::new(Mutex::new(simu));
    let simu = Arc::clone(&simu_for_thread);

    thread::spawn(move || {
        simu_thread(simu_for_thread);
    });

    render::run(
        Box::new(move |size| {
            println!("size: {:?}", size);
            simu.lock().unwrap().on_resize(size);
        }),
        Box::new(render),
    )?;
    Ok(())
}
