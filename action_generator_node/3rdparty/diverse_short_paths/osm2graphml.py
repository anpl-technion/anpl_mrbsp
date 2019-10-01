#!/usr/bin/python

"""
Read graphs in Open Street Maps osm format

Based on osm.py from brianw's osmgeocode
http://github.com/brianw/osmgeocode, which is based on osm.py from
comes from Graphserver:
http://github.com/bmander/graphserver/tree/master and is copyright (c)
2007, Brandon Martin-Anderson under the BSD License
"""


import xml.sax
import copy
import networkx
import sys
import math

def main():
    
    if len(sys.argv) != 5:
        print("./osm2graphml.py <in.osm> <start> <goal> <out.graphml>")
        return
    
    # Read the file
    G = read_osm(sys.argv[1])
    
    # Save graph attributes
    G.graph['start'] = sys.argv[2]
    G.graph['goal'] = sys.argv[3]
    G.graph['space'] = 'RV2'
    
    # Save node coordinates
    minx = miny = float('inf')
    maxx = maxy = -minx
    for n in G.nodes_iter():
        node = G.node[n]
        data = node['data']
        node['xcoord'] = data.lon
        node['ycoord'] = data.lat
        if data.lon < minx:
            minx = data.lon
        if data.lon > maxx:
            maxx = data.lon
        if data.lat < miny:
            miny = data.lat
        if data.lat > maxy:
            maxy = data.lat
        del node['data']
    
    # Recast node coordinates
    for n in G.nodes_iter():
        node = G.node[n]
        node['xcoord'] = 1000*(node['xcoord']-minx)/(maxx-minx)
        node['ycoord'] = 1000*(node['ycoord']-miny)/(maxy-miny)
    
    # Rename nodes sequentially
    renaming = dict(zip(G.nodes_iter(), xrange(G.number_of_nodes())))
    networkx.relabel.relabel_nodes(G, renaming, copy=False)
    
    # Save edge weights
    speed_map = {'unclassified':35, 'service':35, 'footway':5, 'primary_link':30, 'track':25, 'residential':25, 'proposed':1,
                 'secondary_link':30, 'primary':35, 'motorway_link':70, 'cycleway':15, 'motorway':75, 'pedestrian':3, 'tertiary_link':25,
                 'trunk':75, 'steps':1, 'trunk_link':75, 'tertiary':35, 'secondary':35}
    for e in G.edges_iter():
        edge = G.get_edge_data(*e)
        x1 = G.node[e[0]]['xcoord']
        y1 = G.node[e[0]]['ycoord']
        x2 = G.node[e[1]]['xcoord']
        y2 = G.node[e[1]]['ycoord']
        speed = speed_map[edge['data'].tags['highway']]
        edge['weight'] = math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))/speed
        del edge['data']
        del edge['id']
    
    # Remove extraneous info and explicitly label them
    for n in G.nodes_iter():
        node = G.node[n]
        node['coords'] = str(node['xcoord'])+","+str(node['ycoord'])
        node['id'] = str(n)
        del node['xcoord']
        del node['ycoord']
    
    # Export it
    networkx.write_graphml(G, sys.argv[4])

def read_osm(filename_or_stream, only_roads=True):
    """
    Read graph in OSM format from file specified by name or by stream object.

    Parameters
    ----------
    filename_or_stream : filename or stream object

    Returns
    -------
    G : Graph

    Examples
    --------
    >>> G=nx.read_osm(nx.download_osm(-122.33,47.60,-122.31,47.61))
    >>> plot([G.node[n]['data'].lat for n in G], [G.node[n]['data'].lon for n in G], ',')

    """
    osm = OSM(filename_or_stream)
    G = networkx.DiGraph()

    for w in osm.ways.itervalues():
        if only_roads and 'highway' not in w.tags:
            continue
        for u,v in zip(w.nds[:-1], w.nds[1:]):
            G.add_edge(u, v, {'id':w.id, 'data':w})
            if not ('oneway' in w.tags and w.tags['oneway'] == 'yes'):
                G.add_edge(v, u, {'id':w.id, 'data':w})
    for n_id in G.nodes_iter():
        n = osm.nodes[n_id]
        G.node[n_id] = dict(data=n)
    return G


class Node:
    def __init__(self, id, lon, lat):
        self.id = id
        self.lon = lon
        self.lat = lat
        self.tags = {}

class Way:
    def __init__(self, id, osm):
        self.osm = osm
        self.id = id
        self.nds = []
        self.tags = {}

    def split(self, dividers):
        # slice the node-array using this nifty recursive function
        def slice_array(ar, dividers):
            for i in range(1,len(ar)-1):
                if dividers[ar[i]]>1:
                    #print "slice at %s"%ar[i]
                    left = ar[:i+1]
                    right = ar[i:]

                    rightsliced = slice_array(right, dividers)

                    return [left]+rightsliced
            return [ar]

        slices = slice_array(self.nds, dividers)

        # create a way object for each node-array slice
        ret = []
        i=0
        for slice in slices:
            littleway = copy.copy( self )
            littleway.id += "-%d"%i
            littleway.nds = slice
            ret.append( littleway )
            i += 1

        return ret



class OSM:
    def __init__(self, filename_or_stream):
        """ File can be either a filename or stream/file object."""
        nodes = {}
        ways = {}

        superself = self

        class OSMHandler(xml.sax.ContentHandler):
            @classmethod
            def setDocumentLocator(self,loc):
                pass

            @classmethod
            def startDocument(self):
                pass

            @classmethod
            def endDocument(self):
                pass

            @classmethod
            def startElement(self, name, attrs):
                if name=='node':
                    self.currElem = Node(attrs['id'], float(attrs['lon']), float(attrs['lat']))
                elif name=='way':
                    self.currElem = Way(attrs['id'], superself)
                elif name=='tag':
                    self.currElem.tags[attrs['k']] = attrs['v']
                elif name=='nd':
                    self.currElem.nds.append( attrs['ref'] )

            @classmethod
            def endElement(self,name):
                if name=='node':
                    nodes[self.currElem.id] = self.currElem
                elif name=='way':
                    ways[self.currElem.id] = self.currElem

            @classmethod
            def characters(self, chars):
                pass

        xml.sax.parse(filename_or_stream, OSMHandler)

        self.nodes = nodes
        self.ways = ways

        #count times each node is used
        node_histogram = dict.fromkeys( self.nodes.keys(), 0 )
        for way in self.ways.values():
            if len(way.nds) < 2:       #if a way has only one node, delete it out of the osm collection
                del self.ways[way.id]
            else:
                for node in way.nds:
                    node_histogram[node] += 1

        #use that histogram to split all ways, replacing the member set of ways
        new_ways = {}
        for id, way in self.ways.iteritems():
            split_ways = way.split(node_histogram)
            for split_way in split_ways:
                new_ways[split_way.id] = split_way
        self.ways = new_ways

if __name__=="__main__":
    main()
