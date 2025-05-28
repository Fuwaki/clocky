use chrono::offset;
use nalgebra::{Isometry, Rotation, Vector2, point, vector};
use pid::Pid;
use rand::rand_core::le;
use rapier2d::{
    math::Point,
    prelude::{
        BroadPhase, CCDSolver, ColliderBuilder, ColliderHandle, ColliderSet, DefaultBroadPhase,
        Group, ImpulseJointSet, IntegrationParameters, InteractionGroups, IslandManager,
        MassProperties, MultibodyJointSet, NarrowPhase, PhysicsPipeline, RigidBodyBuilder,
        RigidBodyHandle, RigidBodySet, SharedShape,
    },
};
use rayon::prelude::*;
fn generate_circle_points(radius: f32, segments: usize) -> Vec<Point<f32>> {
    let mut points = Vec::with_capacity(segments + 1);
    let angle_step = 2.0 * std::f32::consts::PI / segments as f32;

    for i in 0..segments {
        let angle = angle_step * i as f32;
        points.push(point![radius * angle.cos(), radius * angle.sin()]);
    }

    if let Some(&first) = points.first() {
        points.push(first); // 闭合多边形
    }

    points
}
#[derive(Default, Clone, Copy, Debug)]
pub struct ClockStatus {
    pub hour_angle: f32,
    pub minute_angle: f32,
    pub second_angle: f32,
    pub center: (f32, f32),
    pub hour_width: f32,
    pub hour_length: f32,
    pub minute_width: f32,
    pub minute_length: f32,
    pub second_width: f32,
    pub second_length: f32,
    pub hour_offset: f32,
    pub minute_offset: f32,
    pub second_offset: f32,
}
const BOUNCE: f32 = 0.8;
pub struct Simulation {
    ballset: Vec<(RigidBodyHandle, f32)>,
    size: (u32, u32),
    hour: Option<(RigidBodyHandle, ColliderHandle)>,
    second: Option<(RigidBodyHandle, ColliderHandle)>,
    minute: Option<(RigidBodyHandle, ColliderHandle)>,
    hour_pid: Option<Pid<f32>>,
    second_pid: Option<Pid<f32>>,
    minute_pid: Option<Pid<f32>>,

    clock_status: ClockStatus,
    boundary_radius: f32,

    pipeline: PhysicsPipeline,
    rigidbody_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: Box<dyn BroadPhase>,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    boundary_rigid_body: Option<RigidBodyHandle>,
    boundary_collider: Option<ColliderHandle>,
}

impl Simulation {
    pub fn new() -> Simulation {
        let mut s = Simulation {
            ballset: Vec::new(),
            size: (768, 1034),
            //然后就是物理引擎的一堆东西
            pipeline: PhysicsPipeline::new(),
            rigidbody_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: Box::new(DefaultBroadPhase::new()),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            //边界
            boundary_rigid_body: None,
            boundary_collider: None,
            hour: None,
            second: None,
            minute: None,
            hour_pid: None,
            second_pid: None,
            minute_pid: None,
            boundary_radius: 0.0,
            clock_status: ClockStatus {
                hour_angle: 0.0,
                minute_angle: 0.0,
                second_angle: 0.0,
                center: (0.0, 0.0),
                hour_width: 0.0,
                hour_length: 0.0,
                minute_width: 0.0,
                minute_length: 0.0,
                second_width: 0.0,
                second_length: 0.0,
                hour_offset: 0.0,
                minute_offset: 0.0,
                second_offset: 0.0,
            },
        };
        s
    }
    pub fn add_ball(&mut self, ball: (f32, Vector2<f32>)) {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![ball.1.x, ball.1.y])
            .build();
        let handle = self.rigidbody_set.insert(rigid_body);
        let collider = ColliderBuilder::ball(ball.0)
            .restitution(BOUNCE)
            .density(3.0)
            .friction(0.1)
            .collision_groups(InteractionGroups::new(
                Group::GROUP_1,
                Group::GROUP_1 | Group::GROUP_2,
            ))
            .build();
        //TODO:根据大小计算密度
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigidbody_set);
        self.ballset.push((handle, ball.0));
    }
    pub fn clear_all_balls(&mut self) {
        for (handle, _) in self.ballset.drain(..) {
            self.rigidbody_set.remove(
                handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
        println!("All balls cleared.");
    }
    fn add_boundary(&mut self) {
        self.boundary_radius = self.size.0.min(self.size.1) as f32 / 2.5;
        let points = generate_circle_points(self.boundary_radius, 114);
        let shape = SharedShape::polyline(points, None);
        let collider = ColliderBuilder::new(shape)
            .restitution(BOUNCE)
            .density(1.0)
            .collision_groups(InteractionGroups::new(Group::GROUP_1, Group::GROUP_1))
            .build();
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0])
            .build();
        let rb_handle = self.rigidbody_set.insert(rigid_body);
        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, rb_handle, &mut self.rigidbody_set);
        self.boundary_rigid_body = Some(rb_handle);
        self.boundary_collider = Some(collider_handle);
    }
    fn rebuild_boundary(&mut self) {
        if let Some(collider) = self.boundary_collider {
            self.collider_set
                .remove(collider, &mut self.islands, &mut self.rigidbody_set, false);
        }
        if let Some(rb_handle) = self.boundary_rigid_body {
            self.rigidbody_set.remove(
                rb_handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }

        self.add_boundary();
    }
    fn add_clock(&mut self) {
        self.hour_pid = Some(*Pid::new(0.0, f32::MAX).d(0.3, 10.0).p(3.0, 20.0));
        self.second_pid = Some(*Pid::new(0.0, f32::MAX).d(0.3, 10.0).p(3.0, 20.0));
        self.minute_pid = Some(*Pid::new(0.0, f32::MAX).d(0.3, 10.0).p(3.0, 20.0));
        let mut build_stick = |length, width, offset: f32| {
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0])
                .lock_translations()
                .build();

            let rb_handle = self.rigidbody_set.insert(rigid_body);
            // let collider = ColliderBuilder::cuboid(length, width)
            let collider = ColliderBuilder::capsule_x(length, width)
                .density(3.0)
                .collision_groups(InteractionGroups::new(Group::GROUP_2, Group::GROUP_1))
                .friction(0.0)
                .friction_combine_rule(rapier2d::prelude::CoefficientCombineRule::Min)
                .translation(vector![offset, 0.0])
                .build();
            let collider_handle =
                self.collider_set
                    .insert_with_parent(collider, rb_handle, &mut self.rigidbody_set);

            (rb_handle, collider_handle)
        };
        let radius = self.boundary_radius*0.5;
        self.clock_status.hour_length = radius * 0.618 * 0.8;
        self.clock_status.hour_width = radius*0.1;
        self.clock_status.hour_offset = self.clock_status.hour_length * 0.618;
        self.clock_status.minute_length = radius * 0.618;
        self.clock_status.minute_width = radius * 0.09;
        self.clock_status.minute_offset = self.clock_status.minute_length * 0.618;
        self.clock_status.second_length = radius;
        self.clock_status.second_offset = self.clock_status.second_length * 0.618;
        self.clock_status.second_width = radius * 0.03;

        self.clock_status.center = (self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0);

        self.hour = Some(build_stick(
            self.clock_status.hour_length,
            self.clock_status.hour_width,
            self.clock_status.hour_offset,
        ));
        self.second = Some(build_stick(
            self.clock_status.second_length,
            self.clock_status.second_width,
            self.clock_status.second_offset,
        ));
        self.minute = Some(build_stick(
            self.clock_status.minute_length,
            self.clock_status.minute_width,
            self.clock_status.minute_offset,
        ));

        // let collider=ColliderBuilder::cuboid(hx, hy)
    }
    fn rebuild_clock(&mut self) {
        if let Some((rb_handle, collider)) = self.hour {
            self.collider_set
                .remove(collider, &mut self.islands, &mut self.rigidbody_set, false);
            self.rigidbody_set.remove(
                rb_handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
        if let Some((rb_handle, collider)) = self.second {
            self.collider_set
                .remove(collider, &mut self.islands, &mut self.rigidbody_set, false);
            self.rigidbody_set.remove(
                rb_handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
        if let Some((rb_handle, collider)) = self.minute {
            self.collider_set
                .remove(collider, &mut self.islands, &mut self.rigidbody_set, false);
            self.rigidbody_set.remove(
                rb_handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }

        self.add_clock();
    }
    pub fn set_clock_angle(&mut self, angle: (f32, f32, f32)) {
        let normalize_angle = |mut a: f32| {
            while a < -std::f32::consts::PI {
                a += 2.0 * std::f32::consts::PI;
            }
            while a > std::f32::consts::PI {
                a -= 2.0 * std::f32::consts::PI;
            }
            a
        };
        let angle = (
            normalize_angle(angle.0),
            normalize_angle(angle.1),
            normalize_angle(angle.2),
        );
        // println!("angle: {:?}", angle);

        if let Some((handle, _)) = self.hour {
            let body = self.rigidbody_set.get_mut(handle).unwrap();
            body.set_translation(
                vector![self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0],
                true,
            );
            let error = normalize_angle(angle.0 - body.rotation().angle());
            let output = self
                .hour_pid
                .as_mut()
                .unwrap()
                .next_control_output(-error)
                .output;
            // println!("hour output: {}", output);
            body.set_angvel(output, true);
        }
        if let Some((handle, _)) = self.minute {
            let body = self.rigidbody_set.get_mut(handle).unwrap();
            body.set_translation(
                vector![self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0],
                true,
            );
            let error = normalize_angle(angle.1 - body.rotation().angle());
            let output = self
                .second_pid
                .as_mut()
                .unwrap()
                .next_control_output(-error)
                .output;
            // println!("second output: {}", output);
            body.set_angvel(output, true);
        }
        if let Some((handle, _)) = self.second {
            let body = self.rigidbody_set.get_mut(handle).unwrap();
            body.set_translation(
                vector![self.size.0 as f32 / 2.0, self.size.1 as f32 / 2.0],
                true,
            );
            let error = normalize_angle(angle.2 - body.rotation().angle());
            let output = self
                .minute_pid
                .as_mut()
                .unwrap()
                .next_control_output(-error)
                .output;
            // println!("minute output: {}", output);
            body.set_angvel(output, true);
        }
    }

    pub fn on_resize(&mut self, size: (u32, u32)) {
        self.size = size;
        //注意顺序 因为有些尺寸是在某些函数中计算的
        //写屎了
        //kueen
        self.clear_all_balls();
        self.rebuild_boundary();
        self.rebuild_clock();
        self.random_init_with_percentage(0.8);
        self.cleanup_out_of_bounds();
    }
    fn out_ball_for_render(&self) -> Vec<(f32, f32, f32)> {
        self.ballset
            .iter()
            .filter_map(|(handle, radius)| {
                let body = self.rigidbody_set.get(*handle)?;
                let pos = body.translation();
                Some((pos.x, pos.y, *radius))
            })
            .collect()
    }
    fn out_clock_for_render(&mut self) -> ClockStatus {
        if let Some((handle, _)) = self.hour {
            let body = self.rigidbody_set.get(handle).unwrap();
            self.clock_status.hour_angle = body.rotation().angle();
        }
        if let Some((handle, _)) = self.second {
            let body = self.rigidbody_set.get(handle).unwrap();
            self.clock_status.second_angle = body.rotation().angle();
        }
        if let Some((handle, _)) = self.minute {
            let body = self.rigidbody_set.get(handle).unwrap();
            self.clock_status.minute_angle = body.rotation().angle();
        }
        return self.clock_status.clone();
    }
    pub fn random_init(&mut self, ball_num: u32) {
        use rand::Rng;
        let mut rng = rand::rng();
        for _ in 0..ball_num {
            let x = rng.random_range(0.0..self.size.0 as f32);
            let y = rng.random_range(0.0..self.size.1 as f32);
            let radius = rng.random_range(self.boundary_radius*0.008..self.boundary_radius*0.03);
            self.add_ball((radius, Vector2::new(x, y)));
        }
        self.cleanup_out_of_bounds();
    }
    pub fn random_init_with_percentage(&mut self, percentage: f32) {
        assert!(
            (0.0..=1.0).contains(&percentage),
            "percentage must be between 0.0 and 1.0"
        );
        const K: f32 = 0.03; //k的取值和random_init的球的大小的期望值有关
        let ball_num = self.boundary_radius.powi(2) * percentage * K;
        log::info!("ball_num: {}", ball_num);
        self.random_init(ball_num as u32);
    }
    fn cleanup_out_of_bounds(&mut self) {
        use rayon::prelude::*;

        let indices_to_remove: Vec<usize> = self
            .ballset
            .par_iter()
            .enumerate()
            .filter_map(|(i, (handle, _))| {
                let body = self.rigidbody_set.get(*handle)?;
                let pos = body.translation();
                if (pos.x - self.size.0 as f32 / 2.0).powi(2)
                    + (pos.y - self.size.1 as f32 / 2.0).powi(2)
                    > (self.boundary_radius).powi(2)
                {
                    Some(i)
                } else {
                    None
                }
            })
            .collect();

        for i in indices_to_remove.iter().rev() {
            let (handle, _) = self.ballset.remove(*i);
            self.rigidbody_set.remove(
                handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
    }

    pub fn update(&mut self) -> (Vec<(f32, f32, f32)>, ClockStatus) {
        // let hour_body = self.hour.and_then(|(handle, _)| self.rigidbody_set.get(handle));
        // if let Some(body) = hour_body {
        //     println!("Hour hand position: {:?}", body.translation());
        // }
        self.pipeline.step(
            &vector![0.0, 9.81],
            &self.integration_parameters,
            &mut self.islands,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigidbody_set,
            &mut self.collider_set,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );

        return (self.out_ball_for_render(), self.out_clock_for_render());
    }
}
