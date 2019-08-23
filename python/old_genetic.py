import math
from random import randint, random
import numpy as np
import os
from LHS import myLHS
from mainTest import main

def generateInitialPopulation(bounds, sampleSize):

    # The columns of this matrix are: ribThickness, circleDiameter, skinThickness, numberOfRibs, sparThickness
    tempMatrix = myLHS(bounds, sampleSize)

    # Change the order of the columns
    initialPopulation = [ [row[3], row[2], row[4], row[0], row[1]] for row in tempMatrix ]
    print(initialPopulation)
    return initialPopulation

def evaluateFitness(population, numberOfOutputVariables, loadCase):

    # Initialize the fitness matrix for each offspring. Each column will be an output variable
    fitnessMatrix = np.zeros((len(population), numberOfOutputVariables)).tolist()

    # Compute the analysis for each offspring (row) in the current population (matrix). The chromosomes (design variables) are the columns
    for index, offspring in enumerate(population):
        # Define the variables needed for the analysis
        loadFactor = 1.5
        seedSize = 3.5
        designVariables = {
            'numberOfRibs': offspring[0],
            'skinThickness': offspring[1],
            'sparThickness': offspring[2],
            'ribThickness': offspring[3],
            'circleDiameter': offspring[4]
        }

        # Run the analysis
        results = main(designVariables, loadCase, loadFactor, seedSize)

        # Convert the output variables from results to a list
        resultsList = [ results['totalMass'], results['massRibs'], results['deflection'], results['stress'] ]

        # Set the row in the fitness matrix to the output variables: [total mass of system, mass of ribs, max deflection, max stress]
        fitnessMatrix[index] = resultsList

    print(fitnessMatrix)
    return fitnessMatrix

def selectTopPerformers(population, fitnessMatrix, numberOfParentsDesired):

    # Kill off all offspring that are above a certain amount stress or deflection
    rowsToKill = []
    for index, row in enumerate(fitnessMatrix):

        # If deflection > 60 inches or stress > 73000 psi, eliminate
        if row[2] > 60 or row[3] > 73000:
            rowsToKill.append(index)
        else:
            pass

    print(rowsToKill)

    # Create a new population by killing the rows that fail the critera
    newPopulation = np.delete(population, rowsToKill, axis = 0).tolist()
    newFitness = np.delete(fitnessMatrix, rowsToKill, axis = 0).tolist()

    columnToSort = 5 # this column is the mass column
    # If we kill off all offspring in the population
    if len(newPopulation) == 0:
        # return the original population and fitness
        sortedPopulation = np.concatenate( (population, fitnessMatrix), axis = 1 )
        sortedPopulation = sorted(sortedPopulation, key = lambda offspring: offspring[columnToSort])
        print('Entire population was killed off... returning original population sorted by mass')
    else:
        # Combine the population data (design variables) and their fitness results (output variables), then sort based on mass
        sortedPopulation = np.concatenate( (newPopulation, newFitness), axis = 1 )
        sortedPopulation = sorted(sortedPopulation, key = lambda offspring: offspring[columnToSort])

    return sortedPopulation

def crossover(parents, offspringMatrixSize, numberOfDesignVariables):

    # Initialize the offspring matrix
    offspringMatrix = np.zeros(offspringMatrixSize)

    # Define a random point in the chromosomes (design variables) to perform a crossover at
    crossoverPoint = randint(0, numberOfDesignVariables)

    # Iterate for however many offspring is desired
    for offspringNumber in range(offspringMatrixSize[0]):

        # Define the indexes of the parents to mate
        parent1Index = offspringNumber % len(parents)
        parent2Index = (offspringNumber + 1) % len(parents)

        # The new offspring will take genes from the first parent until the crossover point
        offspringMatrix[offspringNumber][0:crossoverPoint] = parents[parent1Index][0:crossoverPoint]

        # The new offspring will take the rest of the genes from the second parent
        offspringMatrix[offspringNumber][crossoverPoint:] = parents[parent2Index][crossoverPoint:5]

    return offspringMatrix

def mutate(offspringMatrix, bounds):

    # Initialize the mutated offspring matrix
    mutatedOffspring = np.zeros( offspringMatrix.shape )

    # Mutate each offspring with either a small, medium, or large mutation
    for index, row in enumerate(offspringMatrix):
        mutationChance = random()

        mutation = 0.02
        # 63% chance of small mutation
        if mutationChance < 0.95:
            mutation = 0.05

        # 25% chance of medium mutation
        elif mutationChance < 0.32:
            mutation = 0.15

        # 7% chance of large mutation
        elif mutationChance < 0.07:
            mutation = 0.5

        # Mutate the current row
        mutatedRow = [ (element + mutation*random()*element) for element in row ]

        # Add the mutated row to the mutation matrix
        mutatedOffspring[index] = mutatedRow

    return mutatedOffspring

def writeBestPerformersToCSV(bounds, bestPerformers):

    # If the .csv file already exists then delete it
    bestPerformersFile = 'GeneticBestPerformers.csv'
    try:
        os.remove(bestPerformersFile)
    except OSError:
        pass

    # Write the data to the output file
    np.savetxt(bestPerformersFile, bestPerformers, delimiter = ',', fmt = '%0.2f')

    # Write the bounds used to a .txt file
    boundsFile = 'BoundsLHS.txt'
    open(boundsFile, 'w').close()
    DataFile = open(boundsFile, 'w')
    for key in bounds:
        lowerBound = bounds[key][0]
        upperBound = bounds[key][1]

        DataFile.write('%s \nmin: %0.2f \nmid: %0.2f\n\n' % (key, lowerBound, upperBound) )

def writePopulationToOutputFile(populationAndResultsMatrix):

    outputFile = 'PopulationData.csv'
    
    # Write the data to the output file
    np.savetxt(outputFile, populationAndResultsMatrix   , delimiter = ',', fmt = '%0.2f')

def geneticAlgorithim(bounds, numberOfGenerations, sampleSize, numberOfOutputVariables, loadCase):
    
    ### STEP 1: Generate the initial population using a latin hypersquare
    population = generateInitialPopulation(bounds, sampleSize)

    # Best performers has creates a column for each design variable and output variable, and has a row for each generation
    bestPerformers = [[None for _ in range( len(population[0]) + numberOfOutputVariables )] for _ in range(numberOfGenerations)]

    # Write the LHS_Matrix (initial population) to a *.csv file
    # If the .csv file already exists then delete it
    LHS_MatrixFile = 'LHS_Matrix.csv'
    try:
        os.remove(LHS_MatrixFile)
    except OSError:
        pass

    np.savetxt(LHS_MatrixFile, population, delimiter = ',', fmt = '%0.2f')

    # Iterate over the number of generations
    for generation in range(numberOfGenerations):

        ### STEP 2: Evaluate the fitness of the population
        fitnessMatrix = evaluateFitness(population, numberOfOutputVariables, loadCase)

        # Write the population and their results to an output file
        populationAndResults = np.concatenate( (population, fitnessMatrix), axis = 1 ).tolist()
        writePopulationToOutputFile(populationAndResults)

        ### STEP 3: Evaluate the fitness matrix, kill off all offspring that fail the criteria, and grab the top 10 performers
        numberOfParents = 10
        topPerformers = selectTopPerformers(population, fitnessMatrix, numberOfParents)
        print(topPerformers[0])
        bestPerformers[generation] = topPerformers[0].tolist()
        bestPerformers[generation].append(generation) # add the generation number to the last column

        # Since I kinda wrote the code backwards (evaluate and select best performers first), if we're on the last generation there's no need to continue
        # because the data generated wont be evaluated
        if generation == numberOfGenerations:
            break

        ### STEP 4: Create new generations by performing crossover 
        # We want to create offspring with:
        # rows: we want to create offspring until we reached the size of the initial population
        # columns: we want as many columns as we have genes (ie. the number of variables we're varying)
        offspringMatrixSize = (sampleSize - len(topPerformers), len(bounds))
        offspring = crossover(topPerformers, offspringMatrixSize, len(bounds))

        ### STEP 5: Mutate the offspring and create the new population to evaluate
        mutatedOffspring = mutate(offspring, bounds)

        # Create the new population based off the parents and the offspring
        population[ 0 : numberOfParents ][:] = topPerformers[0:len(bounds)]
        population[numberOfParents:][:] = mutatedOffspring

    # Sort the bestPerformers matrix based on mass
    # Has format: stiffener height, stiffener width, stiffener thickness, skin thickness, number of stiffeners, system mass, did stiffener buckle, did plate buckle, 
    columnToSortBy = 5
    bestPerformers = sorted(bestPerformers, key = lambda offspring : offspring[columnToSortBy])

    # Write the data to files
    writeBestPerformersToCSV(bounds, bestPerformers)

    return bestPerformers

loadCase = 'cruise'
numberOfOutputVariables = 4
numberOfGenerations = 3
sampleSize = 4
bounds = {
    'numberOfRibs': [5, 30],
    'skinThickness': [0.025, 0.5],
    'sparThickness': [0.5, 2.0],
    'ribThickness': [0.03, 0.2],
    'circleDiameter': [0.25, 0.5]
}

bestPerformers = geneticAlgorithim(bounds, numberOfGenerations, sampleSize, numberOfOutputVariables, loadCase)