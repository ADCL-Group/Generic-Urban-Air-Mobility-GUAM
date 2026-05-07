function scale_prop_coef_15x55MR(scaling_factor)
    % Propeller data taken from the UIUC propeller data base

    funcDir = fileparts(mfilename('fullpath'));
    
    data1 = load('APC_15x55MR_CSVs/APC_15x55MR_1000.csv');
    data2 = load('APC_15x55MR_CSVs/APC_15x55MR_2000.csv');
    data3 = load('APC_15x55MR_CSVs/APC_15x55MR_3000.csv');
    data4 = load('APC_15x55MR_CSVs/APC_15x55MR_4000.csv');
    data5 = load('APC_15x55MR_CSVs/APC_15x55MR_5000.csv');
    data6 = load('APC_15x55MR_CSVs/APC_15x55MR_6000.csv');
    data7 = load('APC_15x55MR_CSVs/APC_15x55MR_7000.csv');
    data8 = load('APC_15x55MR_CSVs/APC_15x55MR_8000.csv');
    data9 = load('APC_15x55MR_CSVs/APC_15x55MR_9000.csv');
    data10 = load('APC_15x55MR_CSVs/APC_15x55MR_10000.csv');
    data11 = load('APC_15x55MR_CSVs/APC_15x55MR_11000.csv');
    data12 = load('APC_15x55MR_CSVs/APC_15x55MR_12000.csv');
    data13 = load('APC_15x55MR_CSVs/APC_15x55MR_13000.csv');
    data14 = load('APC_15x55MR_CSVs/APC_15x55MR_14000.csv');
    data15 = load('APC_15x55MR_CSVs/APC_15x55MR_15000.csv');
    data16 = load('APC_15x55MR_CSVs/APC_15x55MR_16000.csv');
    
    J = [   data1(:,2); data2(:,2); data3(:,2); data4(:,2);
            data5(:,2); data6(:,2); data7(:,2); data8(:,2);
            data9(:,2); data10(:,2); data11(:,2); data12(:,2);
            data13(:,2); data14(:,2); data15(:,2); data16(:,2)];
    Ct = [  data1(:,4); data2(:,4); data3(:,4); data4(:,4);
            data5(:,4); data6(:,4); data7(:,4); data8(:,4);
            data9(:,4); data10(:,4); data11(:,4); data12(:,4);
            data13(:,4); data14(:,4); data15(:,4); data16(:,4)];
    Cp = [  data1(:,5); data2(:,5); data3(:,5); data4(:,5);
            data5(:,5); data6(:,5); data7(:,5); data8(:,5);
            data9(:,5); data10(:,5); data11(:,5); data12(:,5);
            data13(:,5); data14(:,5); data15(:,5); data16(:,5)];
    
    Ct_p = polyfit(J,Ct,2);
    Cp_p = polyfit(J,Cp,2);
    
    Jb = 0:0.01:1;
    
    Re_factor = scaling_factor^1.5;
    
    Ct_scaled = Ct * Re_factor^(-0.2);
    Cp_scaled = Cp * Re_factor^(-0.1);
    
    Ct_p_scaled = polyfit(J,Ct_scaled,2);
    Cp_p_scaled = polyfit(J,Cp_scaled,2);
    
    APC_15x55MR_scaled_coef = [Ct_p_scaled' Cp_p_scaled'];
    save(fullfile(funcDir, 'APC_15x55MR_scaled_coef.mat'), ...
         'APC_15x55MR_scaled_coef');
end