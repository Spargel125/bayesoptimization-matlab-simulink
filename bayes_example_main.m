%% ベイズ最適化をSimulinkと組み合わせて行うデモ
clear
close all

global pid_p pid_i pid_d simOptions

%% パラメータ設定
timeStep = 0.01; %[s] シミュレーションの周期
% simulink実行オプション設定
simOptions = {'CaptureErrors', 'on','timeout',10}; 

pid_p=5; % Pゲインの初期値 
pid_i=5; % Iゲインの初期値
pid_d=0; % Dゲインの初期値

% 初期応答の取得
initial_out = sim("pid_model.slx",simOptions{:});

%% bayes最適化の設定
% results = bayesopt(fun,vars,Name,Value)
% 最適化用の変数を作成する．
% 変数を混同しないように別名を付ける．
% optimizableVariable('元の変数名',[探索範囲最小値　最大値],'Type','型名');
% 型名は'real' | 'integer' | 'categorical'
bayesopt_pid_p = optimizableVariable('pid_p',[0,50],'Type','real');
bayesopt_pid_i = optimizableVariable('pid_i',[0,25],'Type','real');
bayesopt_pid_d = optimizableVariable('pid_d',[0,10],'Type','real');

% 最適化関数の設定
% 関数ハンドルを作成
% myOptimizationFunctionは自作関数．xは最適化する変数
% miOptimizationFunctionの中でシミュレーションの実行と評価を行う．
optFunction = @(x)myOptimizationFunction(x);


%% ベイズ最適化の実行
% resultに格納される
result = bayesopt( ...
    optFunction, ... % 評価する関数
    [bayesopt_pid_p,bayesopt_pid_i,bayesopt_pid_d], ... % 変数
    'XConstraintFcn',@xconstraint, ...
    'InitialX',table(pid_p,pid_i,pid_d) ... % 初期値．tableで与える必要がある
    ); 


%% 結果の表示
% 最適化後のパラメータ値
disp("最適化後のパラメータ値")
disp(result.XAtMinObjective)
% 最適化後の評価関数値
disp("評価関数値の最小")
disp(result.MinObjective)

% 最適化後の応答の取得
% 変数を取り出す
pid_p = result.XAtMinObjective.pid_p;
pid_i = result.XAtMinObjective.pid_i;
pid_d = result.XAtMinObjective.pid_d;
opt_out = sim("pid_model.slx",simOptions{:});

% プロット
figure()
hold on
plot(initial_out.tout,initial_out.target,'k:',"DisplayName",'target');
plot(initial_out.tout,initial_out.plant_output,'b',"DisplayName",'initial');
plot(opt_out.tout,opt_out.plant_output,'r',"DisplayName",'optimization');
hold off
legend()
xlabel('Time [s]')
ylabel('Output')


%% 関数
function objective=myOptimizationFunction(x)
    % シミュレーションを行い，評価値を返す．
    % 引数：x (探索パラメータ)
    % 返値: objective(評価値）
    % シミュレーション時間は10秒，評価値は1秒~10秒の間での目標値との差とする．

    % simulinkに読み込むため，グローバル変数とする．
    % ※set_paramを使えばglobalにする必要はない．
    global pid_p pid_i pid_d simOptions

    
    % 変数名の取り出し．探索するパラメータはxの中に構造体形式で格納されている．
    pid_p = x.pid_p;
    pid_i = x.pid_i;
    pid_d = x.pid_d;

    % simulinkの実行．
    % 実行
    % simulink desktop-realtimeで外部機器と連携して実行する場合は,
    % out = SLDRT.run(modelName)　
    % とする．
    out = sim("pid_model.slx",simOptions{:});

    % 評価値の計算．
    % 1秒から10秒の範囲で目標値との差の二乗和で評価する
    idx_st = 1*100+1; % 評価範囲のはじめのインデックス
    idx_ed = 10*100+1;% 評価範囲の終わりのインデックス
    objective = sum( ...
        (out.target(idx_st:idx_ed)-out.plant_output(idx_st:idx_ed)).^2 ...
        );
end

% 制約関数
function tf = xconstraint(x)
    % tf=trueであれば制約条件を満たす．
    % 今回は制約条件をpゲインが0より大きいとする．
    tf = x.pid_p>0;
end
