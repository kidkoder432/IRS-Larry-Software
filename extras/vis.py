from dash import Dash, dcc, html
import plotly.express as px
import dash_bootstrap_components as dbc
from dash_bootstrap_templates import load_figure_template
import pandas as pd

df = pd.read_csv("data0328.csv")

load_figure_template("cyborg")

df["TvcX"] -= 100
df["TvcY"] -= 97

df.loc[df["Dt"] > 0.1, "Dt"] = 30

df["TvcX"] = -df["TvcX"]

cols = [
    "Ax",
    "Ay",
    "Az",
    "Gx",
    "Gy",
    "Gz",
    "Roll",
    "Pitch",
    "Yaw",
    "TvcX",
    "TvcY",
    "State",
    "Px",
    "Ix",
    "Dx",
    "Py",
    "Iy",
    "Dy",
]
fig = px.line(df, x="Time", y=cols)
fig.add_scatter(
    x=df["Time"], y=df["Dt"], mode="lines", line=dict(color="white"), name="Dt"
)

app = Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])
app.layout = dbc.Container(
    [dcc.Graph(figure=fig)]
)

if __name__ == "__main__":
    app.run(debug=True)
