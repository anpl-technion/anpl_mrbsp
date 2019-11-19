#!/usr/bin/python

import matplotlib
# Allow matplotlib to generate images without a display
matplotlib.use('Agg')

import matplotlib.pyplot
import networkx
import sys

def main():
    
    if len(sys.argv) != 4:
        print("./draw.py <in.graphml> <paths.txt> <out.png>")
        return
    
    # Read the file
    G = networkx.read_graphml(sys.argv[1])
    for k in ['space', 'start', 'goal']:
        if k in G.graph:
            del G.graph[k]
    
    # Read the path file
    f = open(sys.argv[2], "r")
    paths = map(lambda s: s.split(), f.read().splitlines())
    f.close()
    
    # Save node coordinates and colors
    for n in G.nodes_iter():
        node = G.node[n]
        coords = map(float, node['coords'].split(','))
        node['xcoord'], node['ycoord'] = coords[0], coords[1]
        del node['coords']
        i = path_index_n(paths, n)
        if i > 0:
            node['color'] = rgb(150, 250, 100)
            node['size'] = 8
        else:
            node['color'] = rgb(180, 180, 180)
            node['size'] = 2
    
    # Save edge colors
    edges_redraw = []
    for e in G.edges_iter():
        edge = G.get_edge_data(*e)
        i = path_index_e(paths, e)
        if i > 0:
            edge['color'] = rgb(63+192*i, 128*i, 255-192*i)
            edges_redraw.append(e)
        else:
            edge['color'] = rgb(180, 180, 180)
    
    # Draw it
    pos_map = {n: p for (n,p) in map(lambda n: (n, (G.node[n]['xcoord'], G.node[n]['ycoord'])), G.nodes_iter())}
    ncolors = map(lambda n: G.node[n]['color'], G.nodes_iter())
    nsizes = map(lambda n: G.node[n]['size'], G.nodes_iter())
    ecolors = map(lambda e: G.get_edge_data(*e)['color'], G.edges_iter())
    ecolors_redraw = map(lambda e: G.get_edge_data(*e)['color'], edges_redraw)
    matplotlib.pyplot.figure(None, (50,30))
    networkx.draw_networkx_nodes(G, pos_map, node_size=nsizes, linewidths=0,
                           node_color=ncolors, edge_color=ecolors, width=0.5)
    networkx.draw_networkx_edges(G, pos_map, edge_color=ecolors, width=0.5, arrows=False)
    networkx.draw_networkx_edges(G, pos_map, edgelist=edges_redraw, edge_color=ecolors_redraw, width=0.5, arrows=False)
    matplotlib.pyplot.savefig(sys.argv[3])
    
    """
    # Enable this to save a version of the graph for Gephi
    for n in G.nodes_iter():
        node = G.node[n]
        del node['color']
        del node['size']
    for e in G.edges_iter():
        edge = G.get_edge_data(*e)
        del edge['color']
    networkx.write_graphml(G, "gephi.graphml")
    """

def rgb(r,g,b):
    
    return map(lambda c: c/256.0, (r,g,b))


def path_index_n(paths, n):
    
    if len(paths) == 0:
        return 0
    
    if n == paths[0][0] or n == paths[0][-1]:
        return 1
    return 0

def path_index_e(paths, (u,v)):
    
    if len(paths) == 0:
        return 0
    
    ret = 0
    for p in paths:
        for i in xrange(len(p)-1):
            if (p[i],p[i+1]) == (u,v) or (p[i],p[i+1]) == (v,u):
                ret += 1
                break
    return ret/float(len(paths))

if __name__=="__main__":
    main()
