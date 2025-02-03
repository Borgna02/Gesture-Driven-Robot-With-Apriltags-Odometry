import pandas as pd
import matplotlib.pyplot as plt

# Leggi il dataset dal file CSV
df = pd.read_csv('distanze.csv')



# Crea uno scatter plot con tutti i dati
# plt.figure(figsize=(10, 6))
# plt.scatter(df['distanza_reale'], df['distanza_calcolata'])

# Conta il numero di volte in cui distanza_calcolata è maggiore di distanza_reale
count_greater = (df['distanza_calcolata'] < df['distanza_reale']).sum()

# Stampa il risultato
print(f'Il numero di volte in cui distanza_calcolata è maggiore di distanza_reale: {count_greater}')


plt.scatter(df['distanza_reale'], df['distanza_calcolata'], alpha=0.7, edgecolors='w', s=100)

# Disegna la linea diagonale
min_val = min(df['distanza_reale'].min(), df['distanza_calcolata'].min())
max_val = max(df['distanza_reale'].max(), df['distanza_calcolata'].max())
plt.plot([min_val, max_val], [min_val, max_val], color='orange', linestyle='-', linewidth=2)


# Aggiungi titoli e etichette
plt.title('Scatter Plot: Distanza Calcolata vs Distanza Reale')
plt.xlabel('Distanza Reale')
plt.ylabel('Distanza Calcolata')

# Aggiungi una griglia
plt.grid(True)

# Mostra il grafico
plt.show()





# Raggruppa i dati per distanza_reale e crea una lista di distanza_calcolata per ogni gruppo
grouped = df.groupby('distanza_reale')['distanza_calcolata'].apply(list).reset_index()

# Crea una lista di distanze reali e una lista di liste di distanze calcolate
distanza_reale = grouped['distanza_reale']
distanza_calcolata = grouped['distanza_calcolata']

# Crea il box plot
plt.figure(figsize=(12, 6))
plt.boxplot(distanza_calcolata, labels=distanza_reale, patch_artist=True)

# Aggiungi titoli e etichette
plt.title('Box Plot: Distribuzione di Distanza Calcolata per Distanza Reale')
plt.xlabel('Distanza Reale')
plt.ylabel('Distanza Calcolata')

# Mostra il grafico
plt.show()


# Raggruppa i dati per distanza_reale e calcola la media degli errori per ogni gruppo
grouped = df.groupby('distanza_reale')['errore'].mean().reset_index()

# Estrai i valori come array NumPy
distanza_reale = grouped['distanza_reale'].values
media_errori = grouped['errore'].values

# Crea il grafico
plt.figure(figsize=(10, 6))
plt.plot(distanza_reale, media_errori, marker='o', linestyle='-', color='b')

# Aggiungi titoli e etichette
plt.title('Andamento della Media degli Errori in Funzione della Distanza Reale')
plt.xlabel('Distanza Reale')
plt.ylabel('Media degli Errori')

# Mostra il grafico
plt.grid(True)
plt.show()