#!/usr/bin/env python3
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# 1. Poses captured by the user
# Format: [tx, ty, tz, qx, qy, qz, qw]
poses_robot = [
    [0.1509589226388079, 0.3412956484873212, 0.10667073340084243, -0.4022312127814653, -0.8692240196360811, -0.09314637330059494, 0.27199891230111867], # POSE 1
    [0.22837929848229566, 0.379269787030833, 0.11886941845841847, -0.4967445207375354, -0.8528599923397752, -0.03853434743778371, 0.15617240041370703], # POSE 2
    [0.13904931255704336, 0.37860609694460495, 0.22572483617041242, 0.5384563084647994, 0.8416084797210954, 0.04131143945863308, 0.00730312986759953],  # POSE 3
    [-0.13668722413112108, 0.37885331649995835, 0.24217883670967413, 0.6392358582876551, 0.7546028603480677, 0.14627872649783924, 0.023549412050619615], # POSE 4
    [-0.011466029381632215, 0.40118993356292115, 0.18849062988536686, 0.6229295416767076, 0.734753247883698, 0.15112954453739721, 0.2219376299745561],  # POSE 5
    [-0.036534004819065796, 0.34875747294301757, 0.2754132936611443, 0.6334009962068045, 0.7571912981134502, 0.14947833024517485, 0.05586362727759838], # POSE 6
    [0.03286567066722176, 0.40222446267187084, 0.2055568517774307, 0.5663245733178244, 0.7857296448784461, 0.1400964181260795, 0.2056171112622146]   # POSE 7
]

poses_aruco = [
    [-0.05030024069283788, 0.024736955367977004, 0.31570290055462, -0.7599448891570574, 0.581339788395982, 0.050151785890873275, 0.28637844584350153],  # POSE 1
    [0.07531680641399097, 0.026919325019313208, 0.3089421220419592, -0.7243394683613568, 0.6704614337443116, 0.01620720318443716, 0.15984719891336288],  # POSE 2
    [0.043402887263861975, 0.03588626963554658, 0.3612711088308088, -0.7058595464500154, 0.7068138896896476, 0.04661142679120754, 0.0019496050434199682], # POSE 3
    [-0.1546520882953324, 0.028069494487549307, 0.4380669470628221, -0.5986930814659361, 0.7866228338069876, 0.1508276553101963, 0.006490757612287101],  # POSE 4
    [0.017857310016509077, 0.029311169983156717, 0.33347931011698523, 0.5825251168270371, -0.763794091974839, -0.19797131189393352, 0.19516770480017667],  # POSE 5
    [-0.030493248495305738, 0.03393304428864625, 0.4515768050734512, 0.6022142370726333, -0.7809217861327473, -0.16329986110337724, 0.028850164192086202],  # POSE 6
    [0.03927195642371407, 0.022431125094043464, 0.33277042830934445, 0.6447354561010659, -0.7201711690905657, -0.18292023894879048, 0.1794710702029523]   # POSE 7
]

# Convertir listas a arrays de matrices de rotación y traslación
R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

for pr in poses_robot:
    tx, ty, tz, qx, qy, qz, qw = pr
    # Guardar traslación
    t_gripper2base.append(np.array([[tx], [ty], [tz]]))
    # Convertir cuaternión a matriz de rotación
    r = R.from_quat([qx, qy, qz, qw])
    R_gripper2base.append(r.as_matrix())

for pa in poses_aruco:
    tx, ty, tz, qx, qy, qz, qw = pa
    # Guardar traslación
    t_target2cam.append(np.array([[tx], [ty], [tz]]))
    # Convertir cuaternión a matriz de rotación
    r = R.from_quat([qx, qy, qz, qw])
    R_target2cam.append(r.as_matrix())

# Métodos de OpenCV
methods = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS
}

print("=== RESOLVIENDO CALIBRACIÓN EYE-IN-HAND ===")
for name, method in methods.items():
    try:
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=method
        )
        
        # Calcular los cuaterniones del resultado
        r_res = R.from_matrix(R_cam2gripper)
        q_res = r_res.as_quat() # [qx, qy, qz, qw]
        euler_res = r_res.as_euler('xyz', degrees=True) # [Roll, Pitch, Yaw]
        
        # Calcular la consistencia (reproyectar el marcador a la base del robot para verificar)
        # T_target2base = T_gripper2base(i) * T_cam2gripper * T_target2cam(i)
        target_positions = []
        for i in range(len(poses_robot)):
            T_g2b = np.eye(4)
            T_g2b[:3, :3] = R_gripper2base[i]
            T_g2b[:3, 3] = t_gripper2base[i].flatten()
            
            T_c2g = np.eye(4)
            T_c2g[:3, :3] = R_cam2gripper
            T_c2g[:3, 3] = t_cam2gripper.flatten()
            
            T_t2c = np.eye(4)
            T_t2c[:3, :3] = R_target2cam[i]
            T_t2c[:3, 3] = t_target2cam[i].flatten()
            
            T_t2b = T_g2b @ T_c2g @ T_t2c
            target_positions.append(T_t2b[:3, 3])
            
        target_positions = np.array(target_positions)
        mean_target = np.mean(target_positions, axis=0)
        std_target = np.std(target_positions, axis=0)
        error_norm = np.linalg.norm(std_target)
        
        print(f"\n📌 Método: {name}")
        print(f"  Traslación [X, Y, Z] (metros): [{t_cam2gripper[0][0]:.5f}, {t_cam2gripper[1][0]:.5f}, {t_cam2gripper[2][0]:.5f}]")
        print(f"  Traslación [X, Y, Z] (mm):     [{t_cam2gripper[0][0]*1000:.2f}, {t_cam2gripper[1][0]*1000:.2f}, {t_cam2gripper[2][0]*1000:.2f}]")
        print(f"  Cuaternión [qx, qy, qz, qw]:   [{q_res[0]:.5f}, {q_res[1]:.5f}, {q_res[2]:.5f}, {q_res[3]:.5f}]")
        print(f"  Euler [Roll, Pitch, Yaw] (deg): [{euler_res[0]:.2f}, {euler_res[1]:.2f}, {euler_res[2]:.2f}]")
        print(f"  Desviación Estándar de Reproyección (Error 3D): {error_norm*1000:.2f} mm")
        
    except Exception as e:
        print(f"❌ Error al resolver con método {name}: {e}")
