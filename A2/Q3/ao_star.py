import math

# ---------- Core AO* Functions ----------

def cost_estimate(H, condition, weight=1):
    """Compute AND/OR cost combinations for a given node."""
    costs = {}
    if 'AND' in condition:
        AND_nodes = condition['AND']
        key = ' AND '.join(AND_nodes)
        costs[key] = sum(H[ch] + weight for ch in AND_nodes)
    if 'OR' in condition:
        OR_nodes = condition['OR']
        key = ' OR '.join(OR_nodes)
        costs[key] = min(H[ch] + weight for ch in OR_nodes)
    return costs


def choose_best(H, condition, weight=1):
    """Return minimal estimated cost and chosen decomposition."""
    cdict = cost_estimate(H, condition, weight)
    best_key = min(cdict, key=cdict.get)
    return cdict[best_key], best_key


def reconstruct_path(node, Conditions, H, weight=1):
    """Trace the current best decomposition recursively."""
    if node not in Conditions or not Conditions[node]:
        return node
    cond = Conditions[node]
    costs = cost_estimate(H, cond, weight)
    best_key = min(costs, key=costs.get)
    children = best_key.split()
    if "AND" in best_key:
        left, right = children[0], children[-1]
        return f"{node}->({best_key})[{reconstruct_path(left, Conditions, H)} + {reconstruct_path(right, Conditions, H)}]"
    else:
        next_node = children[0]
        return f"{node}->{reconstruct_path(next_node, Conditions, H)}"


def annotate_nonleaf_stop(start, Conditions, H, weight=1):
    """Print one annotation for a non-leaf node explaining why expansion stopped."""
    path_nodes = []

    def best_key_for(node):
        cond = Conditions.get(node, {})
        if not cond:
            return None, None
        costs = cost_estimate(H, cond, weight)
        k = min(costs, key=costs.get)
        return k, costs

    def walk(node):
        path_nodes.append(node)
        k, _ = best_key_for(node)
        if not k:
            return
        toks = k.split()
        if "AND" in k:
            left, right = toks[0], toks[-1]
            walk(left)
            walk(right)
        else:
            walk(toks[0])

    walk(start)

    # Find first non-leaf node with alternative decompositions
    for node in path_nodes:
        k, costs = best_key_for(node)
        if not k:
            continue
        alts = {kk: vv for kk, vv in costs.items() if kk != k}
        if alts:
            print("\n[Annotation]")
            print(f"Expansion at non-leaf node '{node}' stopped because its chosen decomposition")
            print(f"  {node} -> ({k})")
            print(f"has lower or equal estimated cost than all alternatives:")
            for kk, vv in alts.items():
                print(f"  alt: ({kk})  est_cost={vv:.2f}")
            print(f"Thus H[{node}] fixed at {costs[k]:.2f}.")
            return

    print("\n[Annotation]")
    print("The chosen solution subgraph contains no node with an alternative decomposition;")
    print("expansion stopped because all marked nodes’ children are terminals (already solved).")


def ao_star(start, Conditions, H, weight=1, max_iter=20):
    """Iterative AO* search with selective expansion."""
    iter_count = 0

    while iter_count < max_iter:
        iter_count += 1
        print("=" * 60)
        print(f"Iteration {iter_count}")
        changed = False

        # Traverse bottom-up
        for node in reversed(list(Conditions.keys())):
            cond = Conditions[node]
            if not cond:
                continue  # terminal node
            new_cost, best_rule = choose_best(H, cond, weight)
            if not math.isclose(new_cost, H[node]):
                H[node] = new_cost
                changed = True
            print(f"{node}: best = ({best_rule}), cost = {new_cost:.2f}")

        # Show the current best path from start
        print(f"\nBest path from {start}: {reconstruct_path(start, Conditions, H, weight)}")
        print(f"Current H = {H}\n")

        if not changed:
            print("Converged — AO* reached stable solution graph.")
            annotate_nonleaf_stop(start, Conditions, H, weight)
            break


# ---------- Example 4-node Case ----------

if __name__ == "__main__":
    # Initial heuristic or terminal costs
    H = {'A': -1, 'B': 5, 'C': 2, 'D': 4, 'E': 7, 'F': 9, 'G': 3, 'H': 0, 'I': 0, 'J': 0}

    # AND/OR structure
    Conditions = {
        'A': {'OR': ['B'], 'AND': ['C', 'D']},
        'B': {'OR': ['E', 'F']},
        'C': {'OR': ['G'], 'AND': ['H', 'I']},
        'D': {'OR': ['J']},
        'E': {}, 'F': {}, 'G': {}, 'H': {}, 'I': {}, 'J': {}
    }

    ao_star('A', Conditions, H, weight=1)
