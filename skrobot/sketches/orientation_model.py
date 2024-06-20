import matplotlib.pyplot as plt
import pandas as pd
from sklearn.ensemble import RandomForestRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error
from sklearn.model_selection import train_test_split

df = pd.read_csv('error.csv')
df_senza_phi = pd.read_csv('error_senza_phi.csv')
df_final = pd.read_csv('error_final.csv')


df_orientation_error = pd.read_csv('orientation_error.csv')

df_orientation_error['error'] = abs(
    df_orientation_error['YAW'] - df_orientation_error['real_orientation'])
df_orientation_error['real_orientation'] = round(
    df_orientation_error['real_orientation'], 0)

df_grouped = df_orientation_error.groupby('real_orientation')[
    'error'].mean().reset_index()

print(df_grouped)


# Converting to numpy array for plotting
real_orientation = df_grouped['real_orientation'].values
error = df_grouped['error'].values

# Plotting the data
plt.plot(real_orientation, error)
plt.xlabel('real_orientation')
plt.ylabel('error')
plt.title('Mean Error by Real Orientation')
plt.show()


mae_senza_phi = df_senza_phi['error'].mean()
max_senza_phi = df_senza_phi['error'].max()

mae_final = df_final['error'].mean()
max_final = df_final['error'].max()

print("Mae: ",
      mae_senza_phi, "Max", max_senza_phi, "Mae con dewarping", mae_final, "Max con dewarping", max_final)
