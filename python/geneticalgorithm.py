

class GeneticAlgorithm():

    def __init__(self, number_generations = 0, number_children = 10, number_parents = 10, initial_population = None):
        self.number_generations = number_generations
        self.number_children = number_children
        self.initial_population = initial_population

    def _select_top_performers(self, fitness_matrix, population):

        # Kill off all offspring that don't meet the criteria
        rows_to_kill = []
        # for index, row in enumerate(fitness_matrix):

    
    def run(self, fitness_function):

        if initial_population is None:
            raise ValueError("Please input an initial population.")
    
        # Initialize a matrix to hold the best performer for each generation
        best_performers = [ [ None for _ in len(self.initial_population[0]) ] for _ in self.number_generations ]

        population = self.initial_population
        for generation in range(self.number_generations):

            # STEP 1: Evaluate the fitness of each chromosome in the population
            fitness_matrix = fitness_function(population)
            top_performers = self._select_top_performers(fitness_matrix, population)

            # STEP 2: Has termination criteria been met?
            if generation == self.number_generations:
                return results

            # STEP 3: Select the parents for next generation

            # STEP 4: Perform crossover of parent's chromosomes

            # STEP 5: Mutate each chromosome 