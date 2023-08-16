function [distance, O, tangent_point_AB, tangent_point_BC] = distance_to_tangent_point(r, A, B, C)
    % ����B�㵽�е�ľ������AB��BC���е�Բ��Բ������
    % ����˵����
    % r��Բ�İ뾶
    % A����A�Ķ�ά���꣬��ʽΪ [x, y]
    % B����B�Ķ�ά���꣬��ʽΪ [x, y]
    % C����C�Ķ�ά���꣬��ʽΪ [x, y]

    % ����AB��BC������
    AB_vector = B - A;
    BC_vector = C - B;

    % ����AB��BC�ĳ���
    AB_length = norm(AB_vector);
    BC_length = norm(BC_vector);

    % ����AB��BC�ļн�
    cos_ABC = dot(AB_vector, BC_vector) / (AB_length * BC_length);
    angle_ABC = acos(cos_ABC);

    % ʹ�����Ǻ�������B�㵽�е�ľ���
    distance = r * tan(angle_ABC / 2);

    % �����AB��BC���е�Բ��Բ������
    tangent_point_AB = B - distance * (AB_vector / AB_length);
    tangent_point_BC = B + distance * (BC_vector / BC_length);
    E = (tangent_point_AB + tangent_point_BC)/2;
    BE_dis = sqrt(distance*distance + r*r);
    BE_N = (E-B)/norm(E-B);
    O = B + BE_N*BE_dis;
end