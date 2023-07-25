from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class HoundCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        num_actions = 12
        num_observations = 48

    class terrain( LeggedRobotCfg.terrain ):
        # if rough
        #mesh_type = 'trimesh'
        # if plane
        mesh_type = 'plane'
        measure_heights = False

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.6] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            "FL_roll_joint": 0.0,
            "RL_roll_joint": 0.0,
            "FR_roll_joint": 0.0,
            "RR_roll_joint": 0.0,

            "FL_hip_joint": 0.7854,
            "RL_hip_joint": 0.7854,
            "FR_hip_joint": 0.7854,
            "RR_hip_joint": 0.7854,

            "FL_knee_joint": -1.5708,
            "RL_knee_joint": -1.5708,
            "FR_knee_joint": -1.5708,
            "RR_knee_joint": -1.5708,
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        stiffness = {'joint': 80.}  # [N*m/rad]
        damping = {'joint': 4.}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 5
        use_actuator_network = False
        actuator_net_file = "{LEGGED_GYM_ROOT_DIR}/resources/actuator_nets/anydrive_v3_lstm.pt"
        torque_limit = 200.0

    class asset( LeggedRobotCfg.asset ):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/Hound_new/Hound.urdf"
        name = "hound"
        foot_name = "foot"
        penalize_contacts_on = ["calf", "thigh"]
        terminate_after_contacts_on = ["base", "trunk", "shoulder", "thigh"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        disable_gravity = False
        collapse_fixed_joints = False
        fix_base_link = False
        default_dof_drive_mode = 3
        replace_cylinder_with_capsule = False
        flip_visual_attachments = (
            False
        )

    class domain_rand( LeggedRobotCfg.domain_rand):
        randomize_base_mass = True
        added_mass_range = [-5., 5.]
  
    class rewards( LeggedRobotCfg.rewards ):
        base_height_target = 0.5
        max_contact_force = 500.
        only_positive_rewards = True
        class scales( LeggedRobotCfg.rewards.scales ):
            pass

class HoundCfgPPO( LeggedRobotCfgPPO ):
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'hound' # log folder name
        load_run = -1
