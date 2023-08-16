function plot_circle(A, B, C, O, r, tangent_point_AB, tangent_point_BC)
    % ����������A��B��C��Բ
    % ����˵����
    % A����A�Ķ�ά���꣬��ʽΪ [x, y]
    % B����B�Ķ�ά���꣬��ʽΪ [x, y]
    % C����C�Ķ�ά���꣬��ʽΪ [x, y]
    % O��Բ��Բ�����꣬��ʽΪ [x, y]
    % r��Բ�İ뾶

    % ���Ƶ�A����B����C
    plot(A(1), A(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    hold on;
    plot(B(1), B(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    plot(C(1), C(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    plot(tangent_point_AB(1), tangent_point_AB(2), 'yo', 'MarkerSize', 8, 'LineWidth', 2);
    plot(tangent_point_BC(1), tangent_point_BC(2), 'yo', 'MarkerSize', 8, 'LineWidth', 2);

    % �����߶�AB��BC
    plot([A(1), B(1)], [A(2), B(2)], 'r-', 'LineWidth', 2);
    plot([B(1), C(1)], [B(2), C(2)], 'b-', 'LineWidth', 2);

    % ����Բ
    viscircles(O, r, 'EdgeColor', 'm', 'LineWidth', 1);

    % ����ͼ�εı�����������ǩ
    title('������A��B��C��Բ');
    xlabel('x��');
    ylabel('y��');

    % ����ͼ��
    legend('A', 'B', 'C', 'AB', 'BC', 'Circle');

    % ����ͼ�ε������᷶Χ
    axis equal;
    xlim([min([A(1), B(1), C(1), O(1) - r]) - 2*r, max([A(1), B(1), C(1), O(1) + r]) + 2*r]);
    ylim([min([A(2), B(2), C(2), O(2) - r]) - 2*r, max([A(2), B(2), C(2), O(2) + r]) + 2*r]);

    hold off;
end
