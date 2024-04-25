import sympy as sp

p_3d_x, p_3d_y, p_3d_z = sp.symbols('p_3d_x p_3d_y p_3d_z')
fx, fy, cx, cy, xi, alpha = sp.symbols('fx fy cx cy xi alpha')

# 定义相机参数
cam = {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'xi': xi, 'alpha': alpha}

# 定义函数
def Double_Sphere(p_3d, cam):
    depth_1 = sp.sqrt(p_3d[0]**2 + p_3d[1]**2 + p_3d[2]**2)
    depth_2_tmp_v = (cam['xi']*depth_1 + p_3d[2]) * (cam['xi']*depth_1 + p_3d[2])
    depth_2 = sp.sqrt(p_3d[0]**2 + p_3d[1]**2 + depth_2_tmp_v)

    pix_tmp = cam['alpha'] * depth_2 + (1 - cam['alpha']) * (cam['xi']*depth_1 + p_3d[2])
    p_pix_x = cam['fx'] * p_3d[0] / pix_tmp + cam['cx']
    p_pix_y = cam['fy'] * p_3d[1] / pix_tmp + cam['cy']
    p_pix_z = 1

    return p_pix_x, p_pix_y, p_pix_z

# 计算雅可比矩阵
p_pix_x, p_pix_y, p_pix_z = Double_Sphere((p_3d_x, p_3d_y, p_3d_z), cam)
J = sp.Matrix([p_pix_x, p_pix_y, p_pix_z]).jacobian((p_3d_x, p_3d_y, p_3d_z))
print(J)