function model = autlTrainHybridModel(X_train, y_train, X_val, y_val, rootDir)
% autlTrainHybridModel
% Train hybrid model: Ontology features + AI classifier.

fprintf('[HybridModel] Training Ontology+AI Model...\n');

model = struct();
model.type = 'hybrid_ontology_ai';
model.train_time = datetime('now');

try
    if isempty(ver('statistics'))
        fprintf('[HybridModel] Statistics toolbox not available, using simple SVM\n');
        model.classifier = fitcsvm(X_train, y_train, 'KernelFunction', 'rbf', 'Standardize', true);
    else
        fprintf('[HybridModel] Training neural network (3 layers)...\n');

        net = feedforwardnet([10 5]);
        net.trainFcn = 'trainscg';
        net.layers{1}.transferFcn = 'tansig';
        net.layers{2}.transferFcn = 'tansig';
        net.layers{3}.transferFcn = 'logsig';

        X_train_nn = X_train';
        y_train_nn = y_train' + 1;

        [net, ~] = train(net, X_train_nn, y_train_nn);
        model.classifier = net;
        model.is_nn = true;
    end

    y_pred_train = autlPredictHybrid(model, X_train);
    acc_train = mean(y_pred_train == y_train);

    y_pred_val = autlPredictHybrid(model, X_val);
    acc_val = mean(y_pred_val == y_val);

    fprintf('[HybridModel] Training Accuracy: %.4f\n', acc_train);
    fprintf('[HybridModel] Validation Accuracy: %.4f\n', acc_val);

    model.acc_train = acc_train;
    model.acc_val = acc_val;
    model.y_pred_val = y_pred_val;

    model_dir = fullfile(rootDir, 'data', 'models');
    if ~exist(model_dir, 'dir')
        mkdir(model_dir);
    end

    model_file = fullfile(model_dir, 'model_hybrid_ontology_ai.mat');
    save(model_file, 'model');
    fprintf('[HybridModel] Model saved: %s\n', model_file);

catch ME
    fprintf('[HybridModel] Training failed: %s\n', ME.message);
    model.error = ME.message;
    model.classifier = [];
end

end
