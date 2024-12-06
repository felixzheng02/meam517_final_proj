#!/bin/zsh

# Define the arrays of values
K_values=(1 10 100 1000)
lambda_theta_values=(10 100 1000)

# Loop over the combinations
for K in ${K_values[@]}
do
    for lambda_theta in ${lambda_theta_values[@]}
    do
        echo "Running with K_scalar=$K and lambda_theta=$lambda_theta"
        python test/controller_stablizing_test.py $K $lambda_theta
    done
done
