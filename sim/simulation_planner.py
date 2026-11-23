"""Add a guarded parallel approach to MARCO's simulated clearance batches."""
from tb4_marco_bridge.planning import Planner


class SimulationPlanner(Planner):
    def prepare(self, snapshot, mission):
        # The physical adapter's speed ceiling does not apply in simulation.
        super().prepare(snapshot, {**mission, 'speed_mps': .05})
        self.speed = mission['speed_mps']

    def next_batch(self, snapshot, mission, assignment, completed, intent=None):
        batch = super().next_batch(snapshot, mission, assignment, completed, intent)
        actions = batch['actions']
        if len(actions) != 1 or actions[0]['task'] is not None or not batch['intent']:
            return batch

        relocation = actions[0]
        priority_id = batch['intent']['priority']
        priority = self.names[priority_id]
        if relocation['robot'] == priority:
            return batch

        goal, _ = self.goals[str(batch['intent']['task'])]
        route = self.algorithms.astar(self.grid.cells, self.positions[priority_id], goal)
        if not route or len(route) < 2:
            return batch

        from simulations.ros2.marco_geometry import ROBOT_RADIUS, routes_are_disjoint

        start = snapshot['robots'][priority]['pose']
        points = self.grid.points(route, start)
        peer_routes = [[state['pose'][:2]] for robot, state in snapshot['robots'].items()
                       if robot not in (priority, relocation['robot'])]
        peer_routes.append([point[:2] for point in relocation['points']])
        margin = 2 * (self.dynamic_radius() - ROBOT_RADIUS)
        end = 0
        for i, (a, b) in enumerate(zip(points, points[1:]), start=1):
            if any(not routes_are_disjoint([a[:2], b[:2]], peer, margin)
                   for peer in peer_routes):
                break
            end = i

        # Leave the goal and task completion to the committed priority batch.
        if end:
            actions.append({'robot': priority, 'task': None,
                            'points': self.grid.points(route[:end + 1], start)})
            batch['audit']['parallel_approach'] = priority
        return batch
