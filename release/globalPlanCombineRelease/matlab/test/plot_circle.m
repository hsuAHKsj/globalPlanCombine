function plot_circle(A, B, C, O, r, tangent_point_AB, tangent_point_BC)
    % 绘制三个点A、B、C和圆
    % 参数说明：
    % A：点A的二维坐标，格式为 [x, y]
    % B：点B的二维坐标，格式为 [x, y]
    % C：点C的二维坐标，格式为 [x, y]
    % O：圆的圆心坐标，格式为 [x, y]
    % r：圆的半径

    % 绘制点A、点B、点C
    plot(A(1), A(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    hold on;
    plot(B(1), B(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    plot(C(1), C(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    plot(tangent_point_AB(1), tangent_point_AB(2), 'yo', 'MarkerSize', 8, 'LineWidth', 2);
    plot(tangent_point_BC(1), tangent_point_BC(2), 'yo', 'MarkerSize', 8, 'LineWidth', 2);

    % 绘制线段AB和BC
    plot([A(1), B(1)], [A(2), B(2)], 'r-', 'LineWidth', 2);
    plot([B(1), C(1)], [B(2), C(2)], 'b-', 'LineWidth', 2);

    % 绘制圆
    viscircles(O, r, 'EdgeColor', 'm', 'LineWidth', 1);

    % 设置图形的标题和坐标轴标签
    title('三个点A、B、C和圆');
    xlabel('x轴');
    ylabel('y轴');

    % 设置图例
    legend('A', 'B', 'C', 'AB', 'BC', 'Circle');

    % 设置图形的坐标轴范围
    axis equal;
    xlim([min([A(1), B(1), C(1), O(1) - r]) - 2*r, max([A(1), B(1), C(1), O(1) + r]) + 2*r]);
    ylim([min([A(2), B(2), C(2), O(2) - r]) - 2*r, max([A(2), B(2), C(2), O(2) + r]) + 2*r]);

    hold off;
end
