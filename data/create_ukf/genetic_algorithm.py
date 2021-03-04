import random
import numpy as np
import matplotlib.pyplot as plt
from example import*
from random import randrange

# Number of individuals in each generation 
POPULATION_SIZE = 100

#Run 100 episodes to find fitness of individual
episodes = 100
best_fitness_history = []
generation_timeline = []
  
# Target string to be generated 
goal_fitness = 80
goal_generation = 200

#Length of chromosome (states times possible actions)
chromo_len = 27
chromosome = [] #load best found individuel (chromosome)

#Resolution of random gens (digits in floats)
digits = 5

class Individual(object): 
    ''' 
    Class representing individual in population 
    '''
    def __init__(self, chromosome): 
        self.chromosome = chromosome  
        self.fitness = 0 #self.fitness_ludo_game() #self.cal_fitness()
        self.x0 = []
        self.x1 = []
        self.x2 = []
        self.vision_x = []
        self.vision_y = []
        self.vision_z = []
    
    @classmethod
    def mutated_genes(self):

        #Create random genes for mutation of floats -1 to 1 with 5 digits  
        gene = round(random.uniform(0,1),digits) 
        return gene 
  
    @classmethod
    def create_chromo(self):

        #Create chromosome of genes 
        new_chromo = [0. for i in range(chromo_len)]
        for new_gen in range(chromo_len):
            new_chromo[new_gen] = self.mutated_genes()

        return new_chromo
            
    def mate(self, par2): 
        ''' 
        Perform mating and produce new offspring 
        '''
  
        # chromosome for offspring 
        child_chromosome = [] 
        for gen in range(len(self.chromosome)):

            gp1 = self.chromosome[gen]
            gp2 = par2.chromosome[gen]
  
            # random probability   
            prob = random.random() 
  
            # if prob is less than 0.50, insert gene 
            # from parent 1  
            if prob < 0.50: 
                child_chromosome.append(gp1) 
  
            # if prob is between 0.50 and 1.00, insert 
            # gene from parent 2 
            elif prob < 1.00: 
                child_chromosome.append(gp2) 
  
        #Insert mutated gene to maintain diversity
        rand_gene = randrange(chromo_len)
        child_chromosome[rand_gene] = self.mutated_genes()
        
        rand_gene = randrange(chromo_len)
        child_chromosome[rand_gene] = self.mutated_genes()
        
        rand_gene = randrange(chromo_len)
        child_chromosome[rand_gene] = self.mutated_genes()

        # create new Individual(offspring) using  
        # generated chromosome for offspring 
        return Individual(child_chromosome) 
  
    """
    def cal_fitness(self): 
        ''' 
        Calculate fittness score, it is the number of 
        characters in string which differ from target 
        string. 
        '''
        global TARGET 
        fitness = 0
        for index in range(len(self.chromosome)): 
            if self.chromosome[index] == TARGET[index]:
                fitness+= 1
        return fitness
    """
    def fitness_score(self):
        
        UKF = ukf()
        UKF.covariance = self.chromosome
        UKF.main()

        #print(UKF.covariance)

        mse = 0
        for i in range(len(UKF.new_route_x)-1):
            x_error = abs(UKF.new_route_x[i] - UKF.x0[i])
            y_error = abs(UKF.new_route_y[i] - UKF.x1[i])
            z_error = abs(UKF.new_route_z[i] - UKF.x2[i])
            mse = round(mse,2) + round((x_error + y_error + z_error)/3,2)

        score = 100/(1+round(mse,2)) #Get score from 0 to 100 in % for best fit gene
        print(score)
        
        states = [UKF.x0, UKF.x1, UKF.x2, UKF.mx, UKF.my, UKF.mz]
        return score, states

    def load_chromosome(self,file_name):
        with open(file_name,"r") as ga_file:
            for line in ga_file:
                items = line.split(' ')
                for item in items:
                    chromosome.append(item)
    
    def plot_position(self, fig_name):

        fig, ax = plt.subplots()
        ax.set_axisbelow(True)
        ax.set_facecolor('#E6E6E6')
        plt.grid(color='w', linestyle='solid')

        for spine in ax.spines.values():
            spine.set_visible(False)

        ax.xaxis.tick_bottom()
        ax.yaxis.tick_left()
        ax.tick_params(colors='gray', direction='out')

        for tick in ax.get_xticklabels():
            tick.set_color('gray')
        for tick in ax.get_yticklabels():
            tick.set_color('gray')

        plt.scatter(self.vision_x, self.vision_y, s=10, label='Vision measurements')
        plt.scatter(self.x0, self.x1, s=2, label='EKF Position', c='k')

        # Start/Goal
        plt.scatter(self.vision_x[0], self.vision_y[0], s=60, label='Start', c='g')
        plt.scatter(self.vision_x[-1], self.vision_y[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Estimates UKF and vision position')
        plt.legend(loc='best')
        plt.axis('equal')

        plt.savefig(fig_name)

# Driver code 
def main(): 
    global POPULATION_SIZE 
  
    #current generation 
    generation = 1
  
    found = False
    population = [] 
  
    #Initialization 
    for _ in range(POPULATION_SIZE): 
                chromo = Individual.create_chromo() 
                population.append(Individual(chromo)) 
  
    while not found: 
  
        #Evaluation 
        for i in population:
            fitness_data, _ = i.fitness_score()
            i.fitness = fitness_data
        
        population = sorted(population, key = lambda x:x.fitness, reverse=True) 
  
        #Selection and hence reproduction 
        new_generation = [] 
  
        #Perform Elitism, that mean 10% of fittest population goes to the next generation 
        s = int((10*POPULATION_SIZE)/100) 
        new_generation.extend(population[:s]) 
  
        #From 50% of fittest population, Individuals will mate to produce offspring 
        s = int((90*POPULATION_SIZE)/100) 
        for _ in range(s): 
            parent1 = random.choice(population[:50]) 
            parent2 = random.choice(population[:50]) 
            child = parent1.mate(parent2) 
            new_generation.append(child) 
  
        population = new_generation

        #Solution?
        fitness_data, states = population[0].fitness_score()
        population[0].fitness = fitness_data
        population[0].x0 = states[0]
        population[0].x1 = states[1]
        population[0].x2 = states[2]
        population[0].vision_x = states[3]
        population[0].vision_y = states[4]
        population[0].vision_z = states[5]
        best_fitness_history.append(fitness_data)
        generation_timeline.append(generation)

        print(fitness_data)
        if fitness_data >= goal_fitness:
            found = True
            break
        elif goal_generation == generation:
            found = True
            break

        #Save best individual
        save_best_chromosome = open("ga_data/best_chromosome.txt","w")
        for item in population[0].chromosome:
            save_best_chromosome.write(str(item) + " ")
        save_best_chromosome.close()
        
        print("Generation: {}\tString: {}\tFitness: {}" .\
                format(generation,
                str(population[0].chromosome),
                population[0].fitness))

        fig_name = "Generation: {}\tFitness: {}" .\
                format(generation,
                population[0].fitness)

        population[0].plot_position('ga_data/'+fig_name+'.png') 
        
        generation += 1
        
    fig, ax = plt.subplots()
    
    # use a gray background
    ax.set_axisbelow(True)
    ax.set_facecolor('#E6E6E6')
    # draw solid white grid lines
    plt.grid(color='w', linestyle='solid')

    # hide axis spines
    for spine in ax.spines.values():
        spine.set_visible(False)

    # hide top and right ticks
    ax.xaxis.tick_bottom()
    ax.yaxis.tick_left()

    # lighten ticks and labels
    ax.tick_params(colors='gray', direction='out')
    for tick in ax.get_xticklabels():
        tick.set_color('gray')
    for tick in ax.get_yticklabels():
        tick.set_color('gray')

    ax.plot(generation_timeline,[l[0] for l in best_fitness_history],label='GA performance')
    ax.set_title("GA win succesrate for {} episodes for each generation".format(episodes))
    ax.legend()

    
    plt.ylim((0,episodes))
    plt.xlabel('Generation')
    plt.ylabel('Wins')
    plt.savefig('data/ga_succes_rate.png')
        
    print("Generation: {}\tString: {}\tFitness: {}" .\
          format(generation,
          str(population[0].chromosome),
          population[0].fitness))


if __name__ == '__main__':
    main()
