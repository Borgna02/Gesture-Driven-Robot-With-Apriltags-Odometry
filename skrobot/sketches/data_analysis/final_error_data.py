import pandas as pd
import matplotlib.pyplot as plt


df = pd.read_csv('../position_data_2.csv')  


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