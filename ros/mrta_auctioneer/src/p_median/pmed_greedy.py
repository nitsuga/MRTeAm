"""
pmed_greedy.py

Author Ningchuan Xiao <xiao.37@osu.edu>

https://github.com/gisalgs/optimization/
"""

import sys
from string import atoi

INF = float('inf')


def evaluate(dist, median, n):
    sumdist = 0.0
    p = len(median)
    for i in range(n):
        dist0 = INF
        for j in range(p):
            if dist[i][median[j]] < dist0:
                dist0 = dist[i][median[j]]
        sumdist += dist0
    return sumdist


def pmed_greedy(dist, p):
    n = len(dist)
    median = []
    candidates = [i for i in range(n)]
    for j in range(p):
        dmin = INF
        imin = -1
        for i in candidates:
            d = evaluate(dist, median+[i], n)
            if d < dmin:
                dmin = d
                imin = i
        candidates.remove(imin)
        median.append(imin)

    return median

