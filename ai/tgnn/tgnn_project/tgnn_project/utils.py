
def debug_dataset(dataset):
    for t, snapshot in enumerate(dataset):
        print(f"\n🕒 Snapshot {t}")
        print("Edge Index:")
        print(snapshot.edge_index)

        print("Node Features:")
        print(snapshot.x)
        print("Targets:")
        print(snapshot.y)

        # G = nx.DiGraph()
        # num_nodes = snapshot.x.shape[0]
        # for i in range(num_nodes):
        #     G.add_node(i, feature=snapshot.x[i], target=snapshot.y[i])

        # for src, dst in snapshot.edge_index.T:
        #     G.add_edge(src, dst)

        # pos = nx.spring_layout(G, seed=42)
        # labels = {i: f"{i}\n{snapshot.y[i]:.3f}" for i in range(num_nodes)}
        # nx.draw(G, pos, with_labels=True, labels=labels, node_color='skyblue', edge_color='gray')
        # plt.title(f"Graphe au snapshot {t}")
        # plt.show()
