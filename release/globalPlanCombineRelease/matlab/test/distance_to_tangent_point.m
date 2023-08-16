function [distance, O, tangent_point_AB, tangent_point_BC] = distance_to_tangent_point(r, A, B, C)
    % 计算B点到切点的距离和与AB和BC相切的圆的圆心坐标
    % 参数说明：
    % r：圆的半径
    % A：点A的二维坐标，格式为 [x, y]
    % B：点B的二维坐标，格式为 [x, y]
    % C：点C的二维坐标，格式为 [x, y]

    % 计算AB和BC的向量
    AB_vector = B - A;
    BC_vector = C - B;

    % 计算AB和BC的长度
    AB_length = norm(AB_vector);
    BC_length = norm(BC_vector);

    % 计算AB和BC的夹角
    cos_ABC = dot(AB_vector, BC_vector) / (AB_length * BC_length);
    angle_ABC = acos(cos_ABC);

    % 使用三角函数计算B点到切点的距离
    distance = r * tan(angle_ABC / 2);

    % 求解与AB和BC相切的圆的圆心坐标
    tangent_point_AB = B - distance * (AB_vector / AB_length);
    tangent_point_BC = B + distance * (BC_vector / BC_length);
    E = (tangent_point_AB + tangent_point_BC)/2;
    BE_dis = sqrt(distance*distance + r*r);
    BE_N = (E-B)/norm(E-B);
    O = B + BE_N*BE_dis;
end