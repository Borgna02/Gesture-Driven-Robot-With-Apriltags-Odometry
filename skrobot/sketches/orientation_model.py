import pandas as pd
from sklearn.ensemble import RandomForestRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error
from sklearn.model_selection import train_test_split

df = pd.read_csv('orientation_error.csv')

correlation_matrix = df.corr()
print(correlation_matrix)

X = df.drop('real_orientation', axis=1)
y = df['real_orientation']





old_mae = mean_absolute_error(y, X['YAW'])

import matplotlib.pyplot as plt

plt.scatter(X['YAW'], y)
plt.xlabel('X[YAW]')
plt.ylabel('real_orientation')
plt.title('Scatter Plot')
plt.show()



print(old_mae)