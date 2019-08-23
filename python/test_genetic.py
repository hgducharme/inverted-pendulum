from geneticalgorithm import GeneticAlgorithm
from invertedpendulum import InvertedPendulum
from simulation_and_design import plot_dynamics, get_transfer_functions

import numpy as np
from pyDOE import lhs
from control import *
from control.matlab import ctrb, obsv, ss2tf, step, stepinfo


def fitness_function(population):

    for index, offspring in enumerate(population):

        # Define the chromosomes and build the Q and R matricies from them
        q1 = offspring[0]
        q2 = offspring[1]
        q3 = offspring[2]
        q4 = offspring[3]
        r1 = offspring[4]

        Q = [
            [q1, 0, 0, 0],
            [0, q2, 0, 0],
            [0, 0, q3, 0],
            [0, 0, 0, q4],
        ]

        R = [r1]

        # Solve the Riccati equation and compute K
        feedback_gain, P, closedloop_eigenvalues = lqr(model, Q, R)

        # Create the closed loop system
        closedloop_sys = ss(model.A - (model.B)*feedback_gain, model.B, model.C, model.D)
        theta_tf, cart_tf = get_transfer_functions(closedloop_sys)

        # Evaluate the step_response
        time, cart_and_pendulum_responses = step_response(closedloop_sys)
        plot_dynamics(time, cart_and_pendulum_responses[0], cart_and_pendulum_responses[1])

        for response in cart_and_pendulum_responses:
            steady_state_value = response[-1]

            if steady_state_value < 0:
                supremum = (1. - 0.02) * steady_state_value
                infimum = (1. + 0.02) * steady_state_value

                overshoot = 100. * abs(response.min() - steady_state_value) / abs(steady_state_value - response[0])
            else:
                supremum = (1. + 0.02) * steady_state_value
                infimum = (1. - 0.02) * steady_state_value

                overshoot = 100. * (response.max() - steady_state_value) / (steady_state_value - response[0])
        
            # find Steady State looking for the first point out of specified limits
            for i in reversed(range(time.size)):
                if ( (response[i] <= infimum) | (response[i] >= supremum) ):
                    settling_time = time[i + 1]
                    break

            print(settling_time, overshoot)

if __name__ == "__main__":

    sys = InvertedPendulum(massCart=0.27, massPendulum=0.125, lengthCM=0.15, totalLength=0.30, frictionCoeff=2.5)
    sys.stateSpace = (1)
    model = sys.stateSpace

    # We have 5 design variables in the Q and R matricies: q1, q2, q3, q4, and r1
    latin_hypersquare = lhs(5, 5)

    # Scale each design variable up into a range defined below
    q1_range = [0, 1000]
    q2_range = [0, 300]
    q3_range = [0, 10]
    q4_range = [0, 10]
    r1_range = [0, 100]
    design_variable_bounds = [q1_range, q2_range, q3_range, q4_range, r1_range]
    for i in range(np.size(latin_hypersquare, 1)):
        min_bound = design_variable_bounds[i][0]
        max_bound = design_variable_bounds[i][1]
        range_length = max_bound - min_bound

        latin_hypersquare[:, i] = [ range_length * lhs_element + min_bound for lhs_element in latin_hypersquare[:, i] ]

    genetic = GeneticAlgorithm(number_generations = 10, number_children = 50, number_parents = 10, initial_population = latin_hypersquare)

    # Initialize a matrix to hold the best performer for each generation
    best_performers = [ [ None for _ in range(len(genetic.initial_population[0])) ] for _ in range(genetic.number_generations) ]

    population = latin_hypersquare
    for generation in range(genetic.number_generations):

        # STEP 1: Evaluate the fitness of each chromosome in the population
        fitness_matrix = fitness_function(population)
        # top_performers = select_top_performers(fitness_matrix, population)

        # # STEP 2: Has termination criteria been met?
        # if generation == genetic.number_generations:
        #     return results

        # STEP 3: Select the parents for next generation

        # STEP 4: Perform crossover of parent's chromosomes

        # STEP 5: Mutate each chromosome 