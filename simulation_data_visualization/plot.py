
import pandas as pd
import matplotlib.pyplot as plt
import plotly.express as px

# Read table
table  = pd.read_csv('January_all_runs.csv')
table2 = pd.read_csv('January_all_runs2.csv')
table3 = pd.read_csv('January_all_runs3.csv')
table4 = pd.read_csv('January_all_runs4.csv')

# eliminate data with errors
table = table.loc[table['index'] < 245]
table2 = table2.loc[table2['index'] <= 83]

# concatenate all data and filter based on min distance
table = pd.concat([table, table2, table3, table4], sort=False)
table = table.loc[table['rho'] < 10]


# Plot 2D graphs and save them
alpha = 0.7
table.plot.scatter(x='hesitateTime',
    y='startDelay',
    c='rho', alpha=alpha, colormap='RdPu_r')
plt.gcf().savefig('hesitateTime-startDelay.pdf', bbox_inches='tight')
plt.show()

table.plot.scatter(x='startDelay',
    y='walkDistance',
    c='rho', alpha=alpha, colormap='RdPu_r')
plt.gcf().savefig('startDelay-walkDistance.pdf', bbox_inches='tight')
plt.show()

table.plot.scatter(x='startDelay',
    y='hesitateTime',
    c='rho', alpha=alpha, colormap='RdPu_r')
plt.gcf().savefig('startDelay-hesitateTime.pdf', bbox_inches='tight')
plt.show()


## Plot 3D Graph
fig = px.scatter_3d(table, x='hesitateTime', y='startDelay', z='walkDistance',
              color='rho')
fig.show()

