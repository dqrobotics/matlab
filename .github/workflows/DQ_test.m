clear all;
close all;

NUMBER_OF_RANDOM = 1000;

%% Generate data for unary and binary operators
random_dq_a = random('unif',-10,10,[8 NUMBER_OF_RANDOM]);
random_dq_b = random('unif',-10,10,[8 NUMBER_OF_RANDOM]);

%% Binary
result_of_plus = zeros(8, NUMBER_OF_RANDOM);
result_of_minus = zeros(8, NUMBER_OF_RANDOM);
result_of_times = zeros(8, NUMBER_OF_RANDOM);
result_of_dot = zeros(8, NUMBER_OF_RANDOM);
result_of_cross = zeros(8, NUMBER_OF_RANDOM);
result_of_Ad = zeros(8, NUMBER_OF_RANDOM);
result_of_Adsharp = zeros(8, NUMBER_OF_RANDOM);

%% Unary
result_of_conj = zeros(8, NUMBER_OF_RANDOM);
result_of_sharp = zeros(8, NUMBER_OF_RANDOM);

result_of_normalize = zeros(8, NUMBER_OF_RANDOM);
result_of_translation = zeros(8, NUMBER_OF_RANDOM);
result_of_rotation = zeros(8, NUMBER_OF_RANDOM);
result_of_log = zeros(8, NUMBER_OF_RANDOM);
result_of_exp = zeros(8, NUMBER_OF_RANDOM);
result_of_rotation_axis = zeros(8, NUMBER_OF_RANDOM);
result_of_rotation_angle = zeros(8, NUMBER_OF_RANDOM);

%% Loop
for i=1:NUMBER_OF_RANDOM
    %% Binary
    result_of_plus(:,i) = vec8(DQ(random_dq_a(:,i))+DQ(random_dq_b(:,i)));
    result_of_minus(:,i) = vec8(DQ(random_dq_a(:,i))-DQ(random_dq_b(:,i)));
    result_of_times(:,i) = vec8(DQ(random_dq_a(:,i))*DQ(random_dq_b(:,i)));
    result_of_dot(:,i) = vec8(dot(DQ(random_dq_a(:,i)),DQ(random_dq_b(:,i))));
    result_of_cross(:,i) = vec8(cross(DQ(random_dq_a(:,i)),DQ(random_dq_b(:,i))));
    result_of_Ad(:,i) = vec8(Ad(DQ(random_dq_a(:,i)),DQ(random_dq_b(:,i))));
    result_of_Adsharp(:,i) = vec8(Adsharp(DQ(random_dq_a(:,i)),DQ(random_dq_b(:,i))));
    
    %% Unary
    result_of_conj(:,i) = vec8(conj(DQ(random_dq_a(:,i))));
    result_of_sharp(:,i) = vec8(sharp(DQ(random_dq_a(:,i))));
    
    result_of_normalize(:,i) = vec8(normalize(DQ(random_dq_a(:,i))));
    result_of_translation(:,i) = vec8(translation(normalize(DQ(random_dq_a(:,i)))));
    result_of_rotation(:,i) = vec8(rotation(normalize(DQ(random_dq_a(:,i)))));
    result_of_log(:,i) = vec8(log(normalize(DQ(random_dq_a(:,i)))));
    result_of_exp(:,i) = vec8(exp(get_pure(DQ(random_dq_a(:,i)))));
    result_of_rotation_axis(:,i) = vec8(DQ(rotation_axis(normalize(DQ(random_dq_a(:,i))))));
    result_of_rotation_angle(:,i) = vec8(DQ(rotation_angle(normalize(DQ(random_dq_a(:,i))))));
    
end

save DQ_test_data.mat

quit();

function ret = get_pure(dq)
    ret = DQ(vec6(dq));
end
