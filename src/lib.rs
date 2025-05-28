use std::f32::consts::PI;
pub(crate) use std::{
    iter::once_with,
    sync::{Arc, Mutex, OnceLock},
    thread, vec,
};
// use wasm_thread as thread;
use chrono::{Local, Timelike};
use log::info;
use nalgebra::Isometry;
use rand::rand_core::le;
use simulation::ClockStatus;
use vello::{
    Scene,
    kurbo::{Affine, Circle, Ellipse, Line, RoundedRect, Stroke},
    peniko::Color,
};

mod render;
mod simulation;

static CONTEXT: OnceLock<Mutex<Vec<(f32, f32, f32)>>> = OnceLock::new();
static CLOCK_STATUS: OnceLock<Mutex<ClockStatus>> = OnceLock::new();
fn render(scene: &mut Scene) {
    CONTEXT
        .get_or_init(|| Mutex::new(vec![]))
        .lock()
        .unwrap()
        .iter()
        .for_each(|(x, y, z)| {
            let circle = Circle::new((*x, *y), *z as f64);
            let circle_fill_color = Color::from_rgb8(0x73, 0xb1, 0xc9);
            scene.fill(
                vello::peniko::Fill::NonZero,
                Affine::IDENTITY,
                circle_fill_color,
                None,
                &circle,
            );
        });
    let clock_status = CLOCK_STATUS
        .get_or_init(|| Mutex::new(ClockStatus::default()))
        .lock()
        .unwrap();
    // println!("clock_status: {:?}", clock_status);
    let center = nalgebra::Point2::new(clock_status.center.0, clock_status.center.1);
    let center_circle = Circle::new((center.x, center.y), 10.0);

    // 创建指定角度的方向向量
    let mut draw_clock = |angle: f32, length: f32, offset: f32, width, color| {
        let length = length;
        let offset = offset / length;
        let angle = angle - PI / 2.0;
        let width = width as f32 * 2.0;
        //让数据和rapier统一
        let line_end = center
            + Isometry::<f32, nalgebra::Rotation2<f32>, 2>::rotation(angle)
                * nalgebra::Vector2::new(0.0, length)
                * (1.0 + offset);
        let line_start = center
            - Isometry::<f32, nalgebra::Rotation2<f32>, 2>::rotation(angle)
                * nalgebra::Vector2::new(0.0, length)
                * (1.0 - offset);
        let line = Line::new((line_start.x, line_start.y), (line_end.x, line_end.y));
        scene.stroke(
            &Stroke::new(width as f64),
            Affine::IDENTITY,
            color,
            None,
            &line,
        );
    };

    draw_clock(
        clock_status.hour_angle,
        clock_status.hour_length,
        clock_status.hour_offset,
        clock_status.hour_width,
        Color::from_rgba8(0x1b, 0x5b, 0x7e, 0xff),
    );
    draw_clock(
        clock_status.minute_angle,
        clock_status.minute_length,
        clock_status.minute_offset,
        clock_status.minute_width,
        Color::from_rgba8(0x2b, 0x7e, 0x9c, 114),
    );
    draw_clock(
        clock_status.second_angle,
        clock_status.second_length,
        clock_status.second_offset,
        clock_status.second_width,
        Color::from_rgba8(0x4a, 0x9a, 0xb0, 248),
    );
    drop(draw_clock);
    scene.fill(
        vello::peniko::Fill::NonZero,
        Affine::IDENTITY,
        Color::new([0.7, 0.7, 0.7, 0.6]),
        None,
        &center_circle,
    );
}
fn render_simple(scene: &mut Scene) {
    let circle = Circle::new((200.0, 200.0), 100.0);
    let color = Color::from_rgb8(0x73, 0xb1, 0xc9);
    scene.fill(
        vello::peniko::Fill::NonZero,
        Affine::IDENTITY,
        color,
        None,
        &circle,
    );
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
    let mut hour_angle =
        (hour as f32 % 12.0 + minute as f32 / 60.0 + second as f32 / 3600.0) * 30.0;
    let mut minute_angle = (minute as f32 + second as f32 / 60.0) * 6.0;
    let mut second_angle = (second as f32) * 6.0;
    hour_angle -= 90.0;
    minute_angle -= 90.0;
    second_angle -= 90.0;
    (
        hour_angle.to_radians(),
        minute_angle.to_radians(),
        second_angle.to_radians(),
    )
}

fn simu_thread(simu: Arc<Mutex<simulation::Simulation>>) {
    let mut inited = false;
    loop {
        let mut simu = simu.lock().unwrap();
        if !inited {
            simu.random_init_with_percentage(0.5);
            inited = true;
        }
        let (hour, minute, second, millisecond) = get_time();

        let angles = time_to_clock_angle(hour, minute, second, millisecond);
        simu.set_clock_angle(angles);

        let res = simu.update();
        // info!("{:?}",res);
        let mut guard = CONTEXT.get_or_init(|| Mutex::new(vec![])).lock().unwrap();
        *guard = res.0;
        let mut clock_status = CLOCK_STATUS
            .get_or_init(|| Mutex::new(ClockStatus::default()))
            .lock()
            .unwrap();
        *clock_status = res.1;
        drop(simu);
        thread::sleep(std::time::Duration::from_millis(1));
    }
}

pub fn run_app(event_loop: EventLoop<()>) -> anyhow::Result<()> {
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
        event_loop,
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

use winit::event_loop::EventLoop;
// #[wasm_bindgen]
// pub fn init() {
//     console_error_panic_hook::set_once(); // 捕获 panic 并输出到控制台
// }
#[cfg(target_os = "android")]
pub use winit::platform::android::activity::AndroidApp;

#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(app: AndroidApp) {
    use log::info;
    use winit::platform::android::EventLoopBuilderExtAndroid;

    android_logger::init_once(
        android_logger::Config::default().with_max_level(log::LevelFilter::Info),
    );
    info!("开始");
    let event_loop = EventLoop::with_user_event()
        .with_android_app(app)
        .build()
        .expect("Failed to create event loop");
    run_app(event_loop).expect("Failed to run app");
}
