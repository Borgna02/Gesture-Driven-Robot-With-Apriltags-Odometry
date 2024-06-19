import pandas as pd
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score




df = pd.read_csv('distanze_predette.csv')

errore_massimo_predetto = df['errore_predetto'].max() 
errore_massimo_apriltag = df['errore'].max()


mae_predetto = round(mean_absolute_error(df['distanza_reale'], df['distanza_predetta']), 3)
r2_predetto = round(r2_score(df['distanza_reale'], df['distanza_predetta']), 3)
mse_predetto = round(mean_squared_error(df['distanza_reale'], df['distanza_predetta']), 3)

mae_apriltag = round(mean_absolute_error(df['distanza_reale'], df['distanza_calcolata']), 3)
r2_apriltag = round(r2_score(df['distanza_reale'], df['distanza_calcolata']), 3)
mse_apriltag = round(mean_squared_error(df['distanza_reale'], df['distanza_calcolata']), 3)


print(f'Errore massimo predetto: {errore_massimo_predetto}, Errore massimo apriltag: {errore_massimo_apriltag}, MAE predetto: {mae_predetto}, R2 predetto: {r2_predetto}, MSE predetto: {mse_predetto}, MAE apriltag: {mae_apriltag}, R2 apriltag: {r2_apriltag}, MSE apriltag: {mse_apriltag}')