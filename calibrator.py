#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neural_network import MLPRegressor


# In[ ]:


df = pd.read_csv("tabel2.csv")
df.head()


# In[ ]:


plt.scatter(df['f'], df['c'])
plt.xlabel("camera")
plt.ylabel("robot")


# In[ ]:


# X = df[:, :3]

X = df.values[:, :3]
y = df.values[:, 3:]

X.shape,  y.shape


# In[ ]:


X_train, X_test, y_train, y_test = train_test_split(X, y)


# In[ ]:


scaler = StandardScaler()

# Fit only to the training data
scaler.fit(X_train)

StandardScaler(copy=True, with_mean=True, with_std=True)

# Now apply the transformations to the data:
X_train = scaler.transform(X_train)
X_test = scaler.transform(X_test)

# y_train = scaler.transform(y_train)
# y_test = scaler.transform(y_test)


# In[ ]:


import random

class Chromosome():
    def __init__(self):
        self.fitness = 0
        self.random_layer_size = random.randrange(1, 15)
        self.random_learning_rate_init = round(random.uniform(0.01, 0.1), 2)
        self.random_alpha = round(random.uniform(0.01, 0.1), 2)
        self.random_solver = random.choice(['lbfgs', 'sgd', 'adam'])
        self.random_learning_rate = random.choice(['constant', 'invscaling', 'adaptive'])


# In[ ]:


def generate_population(population_size):
    print("Start generation")
    population = []
    for i in range(0, population_size):
        print("Generated number: " + str(i))
        population.append(Chromosome())
        
    return population


def avg(err_list):
    print(sum(err_list))
    print(len(err_list))
    return sum(err_list)/len(err_list) 


def calculate_fitness(predictions, labels):
    X_errors = []
    y_errors = []
    z_errors = []

    predictions = np.array(predictions)
    labels = np.array(labels)

    for pred, label in zip(predictions, labels):
#        print(str(pred) + " ------ " + str(label))

        X_err = label[0] - pred[0]
        y_err = label[1] - pred[1]
        z_err = label[2] - pred[2]

        X_errors.append(X_err)
        y_errors.append(y_err)
        z_errors.append(z_err)

    print("Klaar met loop")
        
    return avg(X_errors) + avg(y_errors) + avg(z_errors)


# In[ ]:


initial_population = generate_population(50)

iterations = 1

# for _ in range(0, iterations):
    
# test = MLPRegressor()
for chromosome in initial_population:
    print("next")
    model = MLPRegressor(hidden_layer_sizes=(chromosome.random_layer_size,),
                     activation='relu',
                     solver='adam',
                     learning_rate='adaptive',
                     learning_rate_init=chromosome.random_learning_rate_init,
                     alpha=chromosome.random_alpha)
    model.fit(X_train, y_train)
    predictions = model.predict(X_test)

#    print(predictions)

    chromosome.score = calculate_fitness(predictions, y_test)

sorted_pop = sorted(population, key=lambda x: x[0])

sorted_pop


# In[ ]:





# In[ ]:




