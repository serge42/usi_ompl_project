#!/usr/bin/env python

try:
    import graph_tool.all as gt
    graphtool = True
except ImportError:
    print('Failed to import graph-tool. PlannerData will not be analyzed or plotted.')
    graphtool = False

    