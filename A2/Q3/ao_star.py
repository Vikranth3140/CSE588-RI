# ao_star_dict.py

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
    """Return the minimal estimated cost and chosen decomposition."""
    cdict = cost_estimate(H, condition, weight)
    best_key = min(cdict, key=cdict.get)
    return cdict[best_key], best_key


def ao_star(start, Conditions, H, weight=1, max_iter=20):
    """Iterative AO* search with selective expansion."""
    solved = set(node for node, cond in Conditions.items() if not cond)
    iter_count = 0

    while iter_count < max_iter:
        iter_count += 1
        print("=" * 60)
        print(f"Iteration {iter_count}")
        changed = False

        # Traverse bottom-up (like dynamic programming)
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

        # Termination: root cost stable
        if not changed:
            print("Converged — AO* reached stable solution graph.")
            break


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


# ---------- Example Usage ----------

if __name__ == "__main__":
    # Initial heuristic or terminal costs
    H = {'A': -1, 'B': 5, 'C': 2, 'D': 4, 'E': 7, 'F': 9, 'G': 3, 'H': 0, 'I': 0, 'J': 0}

    # AND/OR structure (same as your previous example)
    Conditions = {
        'A': {'OR': ['B'], 'AND': ['C', 'D']},
        'B': {'OR': ['E', 'F']},
        'C': {'OR': ['G'], 'AND': ['H', 'I']},
        'D': {'OR': ['J']},
        'E': {}, 'F': {}, 'G': {}, 'H': {}, 'I': {}, 'J': {}
    }

    ao_star('A', Conditions, H, weight=1)
