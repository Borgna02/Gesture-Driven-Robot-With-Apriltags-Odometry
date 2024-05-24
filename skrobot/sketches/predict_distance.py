from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.isotonic import IsotonicRegression
from sklearn.kernel_ridge import KernelRidge
from sklearn.model_selection import train_test_split
from sklearn.linear_model import BayesianRidge, ElasticNet, HuberRegressor, Lars, Lasso, LinearRegression, OrthogonalMatchingPursuit, PassiveAggressiveRegressor, RANSACRegressor, Ridge, TheilSenRegressor, TweedieRegressor
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error

from sklearn.ensemble import AdaBoostRegressor, BaggingRegressor, ExtraTreesRegressor, GradientBoostingRegressor, HistGradientBoostingRegressor, RandomForestRegressor
from sklearn.neighbors import KNeighborsRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.svm import SVR, NuSVR
from sklearn.tree import DecisionTreeRegressor


def splid_df_by_perc(train_perc: float, X: pd.DataFrame, y: pd.DataFrame):
    """Divides X and y in train and test by train size in percentage.

    Args:
        perc (float): size of train set in percentage.
        X (pd.DataFrame): independent variables dataframe.
        y (pd.DataFrame): dependent variables dataframe.

    Returns:
        X_train (pd.DataFrame): dependent variables train dataframe.
        X_test (pd.DataFrame): dependent variables test dataframe.
        y_train (pd.DataFrame): independent variables train dataframe.
        y_test (pd.DataFrame): independent variables test dataframe.
    """

    # Convert size in number of rows
    train_size = int(len(X) * train_perc)

    # Split dataframes
    X_train = X.iloc[:train_size]
    y_train = y.iloc[:train_size]
    X_test = X.iloc[train_size:]
    y_test = y.iloc[train_size:]

    return X_train, X_test, y_train, y_test


# Carica il dataset
df = pd.read_csv('distanze.csv')

# Seleziona le feature e il target
X = df[['distanza_calcolata', 'tag_center_x', 'tag_center_y']]
y = df['distanza_reale']

# Dividi il dataset in set di addestramento e di test
# X_train, X_test, y_train, y_test = splid_df_by_perc(0.8, X, y)
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42)


def train_and_evaluate(model, model_name, X_train, y_train, X_test, y_test, print=False):
    """Train and evaluate a model on the given data.

    Args:
        model: model to train and evaluate.
        X_train (pd.DataFrame): independent variables train dataframe.
        y_train (pd.DataFrame): dependent variables train dataframe.
        X_test (pd.DataFrame): independent variables test dataframe.
        y_test (pd.DataFrame): dependent variables test dataframe.

    Returns:
        test_rmse (float): root mean squared error on test set.
        test_r2 (float): r-squared score on test set.
        test_mae (float): mean absolute error on test set.
        predictions (np.array): model predictions on test set.
    """

    # Addestra il modello
    model.fit(X_train, y_train)

    # Valuta il modello sul set di test
    test_predictions = model.predict(X_test)
    test_rmse = mean_squared_error(y_test, test_predictions)
    test_r2 = r2_score(y_test, test_predictions)
    test_mae = mean_absolute_error(y_test, test_predictions)

    if print:
        print(f"Metriche {model_name} sul set di test:",
            test_rmse, test_r2, test_mae)

    return test_predictions


# Inizializza il modello di regressione lineare
model = LinearRegression()
# Inizializza il modello di regressione RandomForest
forest_model = RandomForestRegressor(random_state=42)


models = [
    # (LinearRegression(), "Linear Regression"),
    # (Ridge(), "Ridge Regression"),
    # (Lasso(), "Lasso Regression"),
    # (ElasticNet(), "Elastic Net"),
    # (Lars(), "Least Angle Regression"),
    # (OrthogonalMatchingPursuit(), "Orthogonal Matching Pursuit"),
    # (BayesianRidge(), "Bayesian Ridge"),
    # (PassiveAggressiveRegressor(), "Passive Aggressive Regressor"),
    # (HuberRegressor(), "Huber Regressor"),
    # (RANSACRegressor(), "RANSAC Regressor"),
    # (TheilSenRegressor(), "Theil-Sen Regressor"),
    # (KernelRidge(), "Kernel Ridge"),
    # (SVR(), "Support Vector Regressor"),
    (NuSVR(), "Nu Support Vector Regressor"),
    # (GaussianProcessRegressor(), "Gaussian Process Regressor"),
    # (DecisionTreeRegressor(), "Decision Tree Regressor"),
    (RandomForestRegressor(), "Random Forest Regressor"),
    (GradientBoostingRegressor(), "Gradient Boosting Regressor"),
    # (AdaBoostRegressor(), "AdaBoost Regressor"),
    (ExtraTreesRegressor(), "Extra Trees Regressor"),
    (HistGradientBoostingRegressor(), "Histogram-Based Gradient Boosting Regressor"),
    (KNeighborsRegressor(), "K-Neighbors Regressor"),
    # (MLPRegressor(), "Multi-layer Perceptron Regressor"),
    # (TweedieRegressor(), "Tweedie Regressor")
]


df_predetto = pd.DataFrame({'Distanza Reale': y_test, 'Distanza Calcolata': X_test['distanza_calcolata'], 'Errore Calcolata': np.round(
    abs(X_test['distanza_calcolata'] - y_test), 2)})
# Plotta i dati di test e le previsioni del modello

string_output = f"Errore medio calcolato dagli aprilTags: {round(np.mean(df_predetto['Errore Calcolata']), 5)} m"

plt.plot(range(len(y_train), len(y_train) + len(y_test)),
         np.array(y_test), color='blue', label='Test True Values')

for model in models:
    current_predictions = train_and_evaluate(
        model[0], model[1], X_train, y_train, X_test, y_test)
    df_predetto[f'Distanza Predetta {model[1]}'] = np.round(
        current_predictions, 2)
    plt.plot(range(len(y_train), len(y_train) + len(y_test)),
             np.array(current_predictions), linestyle='--', label=f'Test Predictions {model[1]}')
    df_predetto[f'Errore Predetta {model[1]}'] = np.round(abs(current_predictions - y_test), 2)
    
    string_output += f"\nErrore medio predetto {model[1]} : {round(np.mean(df_predetto[f'Errore Predetta {model[1]}']), 5)} m"


# Aggiungi titoli e etichette
plt.title('True Values vs Predictions')
plt.xlabel('Sample Index')
plt.ylabel('Distance Real')
plt.legend()

# Mostra il grafico
# plt.show()


df_predetto.to_csv('distanze_predette.csv', index=False)


print(string_output)