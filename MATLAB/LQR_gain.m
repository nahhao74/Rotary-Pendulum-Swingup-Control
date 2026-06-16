%% =================================================================
%  QUBE-SERVO 2: GA OPTIMIZATION (CATCH AT ALPHA=20, THETA=20)
%  =================================================================
function tune_lqr_ga()
    clear; clc; close all;
    
    %% 1. THÔNG SỐ VẬT LÝ & MÔ HÌNH HÓA
    Rm = 8.4; kt = 0.042; km = 0.042; m_rotor = 0.01; L_r = 0.085;        
    m_p = 0.024; L_p = 0.129; g = 9.81;         
    J_m = 4.0e-6; J_h = 0.6e-6; l_p = L_p / 2;      
    J_p_cm = m_p * (L_p^2) / 12; 
    J_arm_rod = m_rotor * (L_r^2) / 3;
    Jr_total  = J_m + J_h + J_arm_rod + m_p*(L_r^2); 
    Jp_total  = J_p_cm + m_p*(l_p^2);
    Dr = 0.0003; Dp = 0.00005; 
    det_M = Jr_total * Jp_total - (m_p * L_r * l_p)^2;
    B_emf = (kt * km) / Rm;
    Dr_total = Dr + B_emf;
    
    A = zeros(4,4);
    A(1,3) = 1; A(2,4) = 1;
    A(3,2) = (m_p^2 * L_r^2 * l_p * g) / det_M;
    A(3,3) = -(Jp_total * Dr_total) / det_M;
    A(3,4) = -(m_p * L_r * l_p * Dp) / det_M;
    A(4,2) = (m_p * g * l_p * Jr_total) / det_M; 
    A(4,3) = -(m_p * L_r * l_p * Dr_total) / det_M;
    A(4,4) = -(Jr_total * Dp) / det_M;
    
    B = zeros(4,1);
    B(3) = (Jp_total * (kt / Rm)) / det_M;
    B(4) = (m_p * L_r * l_p * (kt / Rm)) / det_M;
    
   %% 2. THIẾT LẬP THUẬT TOÁN GA

    lb = [5.0,  100,  10.0, 100.0, 0.05]; 
    ub = [50.0, 400,  80.0, 300.0, 1.50]; 
    
    options = optimoptions('ga', ...
        'PopulationSize', 60, ...          
        'MaxGenerations', 100, ...         
        'Display', 'iter', ...
        'PlotFcn', @gaplotbestf);
        
    % ĐIỀU KIỆN MÔ PHỎNG: THETA = 20 ĐỘ, ALPHA = 20 ĐỘ
    x0 = [20*pi/180; 20*pi/180; 0; 0]; 
    t_sim = 0:0.002:4; 
    ObjectiveFunction = @(vars) lqr_cost_function(vars, A, B, x0, t_sim);
    fprintf('BẮT ĐẦU CHẠY THUẬT TOÁN GA CHO THỬ THÁCH ALPHA=20, THETA=20 (MỀM TAY QUAY)...\n');
    
    %% 3. CHẠY HÀM GA
    [best_vars, best_cost] = ga(ObjectiveFunction, 5, [], [], [], [], lb, ub, [], options);
    
    %% 4. KẾT QUẢ & IN RA MÀN HÌNH
    Q_opt = diag([best_vars(1), best_vars(2), best_vars(3), best_vars(4)]);
    R_opt = best_vars(5);
    [K_opt, ~, ~] = lqr(A, B, Q_opt, R_opt);
    
    sys_cl = ss(A - B*K_opt, B, eye(4), zeros(4,1));
    [y, t, x] = initial(sys_cl, x0, t_sim);
    u_final = - (K_opt * x')';
    
    fprintf('\n=================================================\n');
    fprintf('   KẾT QUẢ GA MATLAB TỐI ƯU (THETA 20, ALPHA 20) \n');
    fprintf('=================================================\n');
    fprintf('Q Tối ưu: q1=%.1f, q2=%.1f, q3=%.1f, q4=%.1f\n', best_vars(1:4));
    fprintf('R Tối ưu: %.4f\n', R_opt);
    fprintf('Max Voltage (Thực tế): %.2f V\n', max(abs(u_final)));
    
    idx_settle = find(abs(x(:,2)) > 1.0*pi/180, 1, 'last'); % Về < 1 độ
    if isempty(idx_settle), t_s = 0; else, t_s = t(idx_settle); end
    fprintf('Thời gian ổn định Alpha (về < 1.0 độ): %.3f s\n', t_s);
    fprintf('\n--- COPY DÒNG NÀY VÀO LQR_CONTROLLER.CPP ---\n');
    fprintf('static const float K_THETA     = %8.4f;\n', K_opt(1));
    fprintf('static const float K_ALPHA     = %8.4f;\n', K_opt(2));
    fprintf('static const float K_THETA_DOT = %8.4f;\n', K_opt(3));
    fprintf('static const float K_ALPHA_DOT = %8.4f;\n', K_opt(4));
    fprintf('=================================================\n');
    
    %% 5. VẼ ĐỒ THỊ KIỂM CHỨNG
    figure('Name', 'GA Response (Theta=20, Alpha=20 - Softer Arm)', 'Position', [100, 100, 800, 800]);
    
    % Biểu đồ Góc tay quay (Theta)
    subplot(3,1,1);
    plot(t, x(:,1)*180/pi, 'b', 'LineWidth', 2); grid on;
    ylabel('Arm Angle (deg)'); title('Đáp ứng góc tay quay \theta (Mềm hơn để bắt Swing-up)');
    yline(0, 'g--'); 
    
    % Biểu đồ Góc con lắc (Alpha)
    subplot(3,1,2);
    plot(t, x(:,2)*180/pi, 'r', 'LineWidth', 2); grid on;
    ylabel('Pen Angle (deg)'); title('Đáp ứng góc con lắc \alpha (Bắt đầu 20^\circ)');
    yline(1.0, 'g--'); yline(-1.0, 'g--'); ylim([-25 25]);
    
    % Biểu đồ Điện áp (Voltage)
    subplot(3,1,3);
    plot(t, u_final, 'y', 'LineWidth', 1.5); grid on;
    ylabel('Voltage (V)'); title('Điện áp điều khiển');
    ylim([-12 12]);
    xlabel('Time (s)');
    
    disp('--- MA TRẬN HỆ THỐNG A ---');
    disp(A);
    disp('--- MA TRẬN ĐẦU VÀO B ---');
    disp(B);
    disp('--- MA TRẬN TRỌNG SỐ Q (TỐI ƯU BỞI GA) ---');
    disp(Q_opt);
    disp('--- MA TRẬN TRỌNG SỐ R (TỐI ƯU BỞI GA) ---');
    disp(R_opt);
    disp('--- MA TRẬN GAIN LQR K ---');
    disp(K_opt);
end

%% =================================================================
%  HÀM MỤC TIÊU - SOFT PENALTY 
%  =================================================================
function cost = lqr_cost_function(vars, A, B, x0, t_sim)
    Q_mat = diag([vars(1), vars(2), vars(3), vars(4)]);
    R_val = vars(5);
    
    try
        [K, ~, ~] = lqr(A, B, Q_mat, R_val);
        sys_cl = ss(A - B*K, B, eye(4), zeros(4,1));
        [~, t, x] = initial(sys_cl, x0, t_sim);
        u = - (K * x')'; 
        
        max_u = max(abs(u));
        idx_settle = find(abs(x(:,2)) > 1.0*pi/180, 1, 'last'); 
        if isempty(idx_settle), t_settle = 0; else, t_settle = t(idx_settle); end
        
        iae_alpha = sum(abs(x(:,2)));
        iae_theta = sum(abs(x(:,1)));
        
        % SOFT PENALTY
        if max_u > 12.0
            penalty = (max_u - 12.0) * 800; 
        else
            penalty = 0;
        end
        
      
        cost = (t_settle * 800) + (iae_alpha * 200) + (iae_theta * 100) + penalty + (max_u * 10);
    catch
        cost = 1e6; 
    end
end