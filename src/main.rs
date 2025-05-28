#![cfg_attr(windows, windows_subsystem = "windows")]
fn main() -> anyhow::Result<()> {
    #[cfg(not(target_os = "android"))]
    {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
        use winit::event_loop::EventLoop;
        let event_loop = EventLoop::new()?;

        clocky::run_app(event_loop)?;
        return Ok(())
    }
    #[cfg(target_os = "android")]
    unreachable!();
}



