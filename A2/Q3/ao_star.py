from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Iterable
import math

# Graph structures
@dataclass
class Hyperedge:
    cost: float
    children: List[str]

@dataclass
class Node:
    name: str
    heuristic: float = 0.0            # h(n), must be >= 0
    terminal_cost: Optional[float] = None  # If set, node is a solved terminal with this exact cost
    edges: List[Hyperedge] = field(default_factory=list)

    # AO* bookkeeping (updated during search)
    solved: bool = False
    best_edge_idx: Optional[int] = None     # index in self.edges that is currently best
    f: float = field(default_factory=lambda: math.inf)  # current best cost estimate

class AOStar:
    def __init__(self, nodes: Dict[str, Node], root: str):
        self.nodes = nodes
        self.root = root
        # Initialize terminals
        for n in self.nodes.values():
            if n.terminal_cost is not None:
                n.solved = True
                n.f = n.terminal_cost
            else:
                # non-terminals start with admissible h(n) (or inf if no heuristic provided)
                n.f = float(n.heuristic) if n.heuristic is not None else math.inf

    # Utility
    def _edge_cost_estimate(self, edge: Hyperedge) -> float:
        return edge.cost + sum(self.nodes[c].f for c in edge.children)

    def _choose_best_edge(self, node: Node) -> Tuple[float, Optional[int]]:
        best_cost = math.inf
        best_idx = None
        for i, e in enumerate(node.edges):
            cost_i = self._edge_cost_estimate(e)
            if cost_i < best_cost:
                best_cost, best_idx = cost_i, i
        return best_cost, best_idx

    def _mark_best_solution_subgraph(self) -> None:
        for n in self.nodes.values():
            n.best_edge_idx = None  # reset marks
        # DFS from root following currently best edges greedily
        def mark(nm: str):
            node = self.nodes[nm]
            if node.solved or not node.edges:
                return
            best_cost, best_idx = self._choose_best_edge(node)
            node.best_edge_idx = best_idx
            for ch in node.edges[best_idx].children:
                mark(ch)
        mark(self.root)

    def _select_frontier_tip(self) -> Optional[str]:
        # Collect frontier nodes along the marked solution subgraph
        frontier = []  # list of (depth, f, name)
        def dfs(nm: str, depth: int):
            node = self.nodes[nm]
            if node.solved:
                return
            if node.best_edge_idx is None:
                # no chosen edge -> this is a tip (leaf in the marked subgraph)
                frontier.append((depth, node.f, nm))
                return
            # otherwise, follow the chosen hyperedge
            for ch in self.nodes[nm].edges[node.best_edge_idx].children:
                dfs(ch, depth + 1)
        dfs(self.root, 0)
        if not frontier:
            return None
        # Prefer deepest tip, tie-break by lower f
        frontier.sort(key=lambda t: (-t[0], t[1], t[2]))
        return frontier[0][2]

    def _backup(self) -> None:
        changed = True
        while changed:
            changed = False
            # Process nodes in reverse topological order if possible; here we just iterate
            for n in self.nodes.values():
                old_f, old_best = n.f, n.best_edge_idx
                if n.solved:
                    continue
                if not n.edges:
                    # Nonterminal but no outgoing edges -> dead end
                    n.f = math.inf
                    n.best_edge_idx = None
                else:
                    est, idx = self._choose_best_edge(n)
                    n.f = min(n.f, est) if n.best_edge_idx is None else est
                    n.best_edge_idx = idx
                    # If all children of current best edge are solved, this node becomes solved
                    if all(self.nodes[ch].solved for ch in n.edges[idx].children):
                        n.solved = True
                        n.f = est
                changed |= (n.f != old_f or n.best_edge_idx != old_best)

    # Pretty printing
    def _edge_to_str(self, n: Node, idx: int) -> str:
        e = n.edges[idx]
        kind = "AND" if len(e.children) > 1 else "OR"
        child_str = "+".join(e.children) if kind == "AND" else e.children[0]
        marker = "*" if n.best_edge_idx == idx else " "
        return f"{marker} {n.name} -[{kind}; c={e.cost}]-> {child_str}  (edge_est={self._edge_cost_estimate(e):.2f})"

    def print_partial_solution(self, iteration: int) -> None:
        print("=" * 72)
        print(f"Iteration {iteration}: partial solution (current f(root)={self.nodes[self.root].f:.2f})")
        print("- Marked edges (*) indicate the current best decomposition from the root.")
        # Show only nodes reachable from the root via marked choices + their siblings
        seen = set()
        def show(nm: str):
            if nm in seen:
                return
            seen.add(nm)
            n = self.nodes[nm]
            tc = f" (terminal={n.terminal_cost})" if n.terminal_cost is not None else ""
            print(f"[Node {n.name}] f={n.f:.2f}{tc} solved={n.solved}")
            if not n.edges:
                return
            for i,_ in enumerate(n.edges):
                print("  " + self._edge_to_str(n, i))
            if n.best_edge_idx is not None:
                for ch in n.edges[n.best_edge_idx].children:
                    show(ch)
        show(self.root)

    # Main loop
    def run(self, max_iters: int = 100) -> None:
        for it in range(1, max_iters + 1):
            self._mark_best_solution_subgraph()
            tip = self._select_frontier_tip()
            self.print_partial_solution(it)
            if self.nodes[self.root].solved:
                print("Root is solved. Stopping.")
                # One-shot annotation: explain one non‑leaf stop
                self._annotate_non_leaf_stop()
                return
            if tip is None:
                print("No expandable frontier. Stopping (no progress possible).")
                self._annotate_non_leaf_stop()
                return
            # Expand the chosen tip (graph is static here; no op) and backup
            self._backup()
        print("Reached iteration limit.")
        self._annotate_non_leaf_stop()

    def _annotate_non_leaf_stop(self) -> None:
        # Find a non-leaf node that is effectively closed because its chosen children are solved
        for nm in self._topdown_nodes():
            n = self.nodes[nm]
            if n.edges and n.best_edge_idx is not None:
                kids = n.edges[n.best_edge_idx].children
                if all(self.nodes[k].solved for k in kids):
                    kind = "AND" if len(kids) > 1 else "OR"
                    est = self._edge_cost_estimate(n.edges[n.best_edge_idx])
                    print("\n[Annotation]")
                    print(f"Expansion at non‑leaf node '{n.name}' stopped because its chosen {kind} decomposition")
                    print(f"  {n.name} -> {('+'.join(kids) if kind=='AND' else kids[0])}")
                    print("has all children solved; any alternative decomposition has >= estimated cost.")
                    # Show alternative(s)
                    alts = [i for i in range(len(n.edges)) if i != n.best_edge_idx]
                    if alts:
                        alt_strs = []
                        for i in alts:
                            e = n.edges[i]
                            alt_strs.append(f"{('AND' if len(e.children)>1 else 'OR')} to {('+'.join(e.children) if len(e.children)>1 else e.children[0])} with est={self._edge_cost_estimate(e):.2f}")
                        print("Alternatives were: " + "; ".join(alt_strs))
                    print(f"Thus f({n.name}) fixed at {n.f:.2f} = chosen_edge_cost_est {est:.2f}.")
                    return

    def _topdown_nodes(self) -> Iterable[str]:
        seen = set()
        order = []
        def dfs(nm: str):
            if nm in seen:
                return
            seen.add(nm)
            order.append(nm)
            n = self.nodes[nm]
            if n.best_edge_idx is not None:
                for ch in n.edges[n.best_edge_idx].children:
                    dfs(ch)
        dfs(self.root)
        # add remaining
        for nm in self.nodes:
            if nm not in seen:
                order.append(nm)
        return order

# Demo: 4-node examplex

def make_4node_example() -> Tuple[Dict[str, Node], str]:
    nodes: Dict[str, Node] = {
        "S": Node("S", heuristic=0.0),
        "A": Node("A", heuristic=1.0),
        "B": Node("B", terminal_cost=2.0),
        "C": Node("C", terminal_cost=4.0),
    }
    # Define hyperedges
    nodes["S"].edges = [
        Hyperedge(cost=1.0, children=["A"]),          # OR
        Hyperedge(cost=2.0, children=["B", "C"]),    # AND
    ]
    # A can: (i) finish with fixed 6 (model as edge to a dummy terminal via cost),
    # or (ii) reduce to C with extra cost 1.
    # We will model (i) as an edge with cost 6 to an empty child set by introducing
    # a zero-cost terminal child "A_done" to keep the uniform representation.
    nodes["A_done"] = Node("A_done", terminal_cost=0.0)
    nodes["A"].edges = [
        Hyperedge(cost=6.0, children=["A_done"]),   # OR: pay 6 and stop
        Hyperedge(cost=1.0, children=["C"]),        # OR: delegate to C with +1
    ]
    return nodes, "S"


def main():
    nodes, root = make_4node_example()
    planner = AOStar(nodes, root)
    planner.run(max_iters=50)

if __name__ == "__main__":
    main()
