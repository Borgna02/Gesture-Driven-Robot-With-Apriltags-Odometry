import pandas as pd
import matplotlib.pyplot as plt


df = pd.read_csv('../position_errors.csv')  

print(f"Errore medio: {df['error'].mean()}, Errore minimo: {df['error'].min()}, Errore massimo: {df['error'].max()}")

df['error'] = round(df['error'], 2)
print(df)

df_grouped_by_error = df.groupby('error')['error'].count().reset_index(name='count')
print(df_grouped_by_error)

plt.figure (figsize=(12, 6))
plt.plot(df_grouped_by_error['error'].to_numpy(), df_grouped_by_error['count'].to_numpy(), marker='.')
plt.title('Distribuzione degli errori')
plt.xlabel('Errore')
plt.ylabel('Frequenza')
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()


# Arrotonda phi a 0 decimali
df['phi'] = round(df['phi'], 0)

# Raggruppa per phi e calcola la media degli errori
df_grouped = df.groupby('phi')['error'].mean().reset_index()

# Rinomina la colonna per chiarezza
df_grouped = df_grouped.rename(columns={'error': 'mean_error'})

# Creazione del grafico
plt.figure(figsize=(12, 6))  # Imposta la dimensione del grafico

# Converti le colonne in array NumPy e crea il plot
plt.plot(df_grouped['phi'].to_numpy(), df_grouped['mean_error'].to_numpy(), marker='.')

# Personalizza il grafico
plt.title('Errore Medio in funzione di Phi')
plt.xlabel('Phi (gradi)')
plt.ylabel('Errore Medio')

# Aggiungi una griglia per migliorare la leggibilità
plt.grid(True, linestyle='--', alpha=0.7)

# Ruota le etichette sull'asse x se necessario per evitare sovrapposizioni
plt.xticks(rotation=45)

# Mostra il grafico
plt.tight_layout()  # Aggiusta automaticamente il layout
plt.show()

# Raggruppa il dataframe in base al valore assoluto di phi
df['abs_phi'] = abs(df['phi'])

# Definisci gli intervalli di raggruppamento
intervals = [(-float('inf'), -30), (-30, -20), (-20, -10), (-10, -5), (-5, 5), (5, 10), (10, 20), (20, 30), (30, float('inf'))]

# Calcola la media degli errori per ogni intervallo
mean_errors = []
for interval in intervals:
    start, end = interval
    mean_error = df[(df['abs_phi'] >= start) & (df['abs_phi'] < end)]['error'].mean()
    mean_errors.append(mean_error)

# Crea un nuovo dataframe con gli intervalli e le medie degli errori
df_grouped_intervals = pd.DataFrame({'Interval': intervals, 'Mean Error': mean_errors})

# Stampa il dataframe raggruppato per intervalli
print(df_grouped_intervals)