''' Plot 2D (x,y) confidence region given 2 matrices X and Y'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from scipy.stats.distributions import chi2

def plot_2D_confidence_region(ax, X, Y, conf_percent, Alpha=0.5, FaceColor='tab:blue', EdgeColor='None', LineStyle='None'): # spacing, facecolor, alpha, edgecolor, linestyle):
    x1m=X.mean(0)
    x2m=Y.mean(0)
    N = X.shape[1]
    Zstar=chi2.ppf(conf_percent, df=2)
    for ii in range(N):
        cv= np.cov(X[:, ii], Y[:, ii])
        cv = cv * Zstar
        [eigenValues, eigenVectors]=np.linalg.eig(cv)
        idx = eigenValues.argsort()[::-1]
        eigenValues = eigenValues[idx]
        if np.min(eigenValues) != eigenValues[1]:
            print("ERROR! Eigs are not in correct (decending) order")
            break
        
        eigenVectors = eigenVectors[:, idx]
        max_eigvec = eigenVectors[:, 0]
        xcenter = x1m[ii]
        ycenter = x2m[ii]
        width = np.sqrt(eigenValues[0])*2
        height = np.sqrt(eigenValues[1])*2
        if eigenVectors[0, 0] == 0:
            rot_angle = 0
        else:
            rot_angle = np.arctan(max_eigvec[1]/max_eigvec[0]) # check if using correct eigvec (corr to max eigval)
            rot_angle = np.rad2deg(rot_angle)
        e1 = patches.Ellipse((xcenter, ycenter), width, height, angle=rot_angle,
                             alpha=Alpha, facecolor=FaceColor, edgecolor=EdgeColor, linestyle=LineStyle, zorder=1)
        ax.add_patch(e1)
        plt.xlabel("$x$", fontsize=12, math_fontfamily='cm')
        plt.ylabel("$y$", fontsize=12, math_fontfamily='cm')
        # set_linestyle('none')
        # conf_reg_2d=patch(x,y,facecolor)
        # conf_reg_2d.EdgeColor = edgecolor
        # conf_reg_2d.LineStyle = linestyle
        # conf_reg_2d.FaceColor = facecolor
        # conf_reg_2d.FaceAlpha = alpha
    return ax

