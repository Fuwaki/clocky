pub(crate) use std::{
    iter::once_with,
    sync::{Arc, Mutex, OnceLock},
    thread, vec,
};
// use wasm_thread as thread;
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
    // println!("hour: {}, minute: {}, second: {}, millisecond: {}", hour, minute, second, millisecond);
    let mut hour_angle = (hour as f32 % 12.0 + minute as f32 / 60.0 + second as f32 / 3600.0) * 30.0;
    let mut minute_angle = (minute as f32 + second as f32 / 60.0) * 6.0;
    let mut second_angle = (second as f32 ) * 6.0;
    hour_angle-=90.0;
    minute_angle-=90.0;
    second_angle-=90.0;

    (hour_angle.to_radians(), minute_angle.to_radians(), second_angle.to_radians())
}

fn simu_thread(simu: Arc<Mutex<simulation::Simulation>>) {
    let mut inited = false;
    loop {
        let mut simu = simu.lock().unwrap();
        if !inited {
            simu.random_init(6000);
            inited = true;
        }
        let (hour, minute, second, millisecond) = get_time();

        let angles = time_to_clock_angle(hour, minute, second, millisecond);
        simu.set_clock_angle(angles);

        let res = simu.update();
        // println!("{:?}",res);
        let mut guard = CONTEXT.get_or_init(|| Mutex::new(vec![])).lock().unwrap();
        *guard = res;
        drop(simu);
        thread::sleep(std::time::Duration::from_millis(1));
    }
}

pub fn run_app() -> anyhow::Result<()> {
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

// use wasm_bindgen::prelude::wasm_bindgen;
// use wasm_bindgen_futures;
// use web_sys::console;
// #[wasm_bindgen]
// pub fn start(){
//     console::log_1(&"helloworld".into());
//     run_app().unwrap();
// }
// use console_error_panic_hook;

// #[wasm_bindgen]
// pub fn init() {
//     console_error_panic_hook::set_once(); // 捕获 panic 并输出到控制台
// }
