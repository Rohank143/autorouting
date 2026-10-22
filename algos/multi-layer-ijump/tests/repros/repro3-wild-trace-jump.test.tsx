import { getSimpleRouteJson } from "solver-utils"
import { test, expect } from "bun:test"
import { convertCircuitJsonToPcbSvg } from "circuit-to-svg"
import { Circuit } from "@tscircuit/core"
import { MultilayerIjump } from "../../MultilayerIjump"

const OneByOnePad = (props: {
  name: string
  pcbX?: number
  pcbY?: number
  layer: string
}) => (
  <chip name={props.name} pcbX={props.pcbX} pcbY={props.pcbY}>
    <footprint>
      <smtpad
        pcbX={0}
        pcbY={0}
        shape="rect"
        width="1mm"
        height="1mm"
        layer={props.layer as any}
        portHints={["pin1"]}
      />
    </footprint>
  </chip>
)

/**
 * https://github.com/tscircuit/autorouting/issues/92
 *
 * When a node hits an obstacle and a travel direction is completely clear
 * (wallDistance === Infinity), the solver previously pushed a candidate with
 * travelDistance = distAlongDir(node, goal) without checking isGoalInTravelDir.
 * Because distAlongDir returns an unsigned absolute distance, whenever the goal
 * was actually behind that travel direction, the router teleported the candidate
 * in the opposite direction away from the goal into empty space (a "wild jump").
 */

test("repro3: no wrong-direction goal-snap candidates generated after hitting a wall", () => {
  const input: any = {
    layerCount: 2,
    minTraceWidth: 0.15,
    obstacles: [
      {
        type: "rect",
        layers: ["top", "bottom"],
        center: { x: 1, y: 0 },
        width: 1,
        height: 2,
        connectedTo: [],
      },
    ],
    connections: [
      {
        name: "n1",
        pointsToConnect: [
          { x: -1, y: 0, layer: "top" },
          { x: 3, y: -1.7, layer: "bottom" },
        ],
      },
    ],
    bounds: { minX: -5, maxX: 5, minY: -5, maxY: 5 },
  }

  const autorouter = new MultilayerIjump({ input, VIA_COST: 1 })
  autorouter.startNode = { x: -1, y: 0, l: 0 } as any
  autorouter.goalPoint = { x: 3, y: -1.7, l: 1 } as any
  autorouter.obstacles = autorouter.createObstacleList({
    connection: input.connections[0],
    obstaclesFromTraces: [],
  }) as any

  const wall = autorouter.obstacles.getObstaclesOverlappingRegion({
    minX: 0,
    minY: -1,
    maxX: 2,
    maxY: 1,
    l: 0,
  })[0]
  const parent = { x: -1, y: 0, l: 0, g: 0 } as any
  const node = {
    x: 0.35,
    y: 0,
    l: 0,
    g: 1.35,
    obstacleHit: wall,
    parent,
  } as any

  const neighbors = autorouter.getNeighbors(node)

  // The goal is at y = -1.7 while the open direction is +y.
  // The buggy code generated a candidate at y = +1.7 (the unsigned projection of the goal onto
  // the wrong direction). No candidate may land on the mirror of the goal in the wrong direction.
  const wild = neighbors.filter(
    (n: any) =>
      (n.y !== node.y &&
        Math.abs(n.y - (2 * node.y - autorouter.goalPoint.y)) < 0.01) ||
      (n.x !== node.x &&
        Math.abs(n.x - (2 * node.x - autorouter.goalPoint.x)) < 0.01),
  )
  expect(wild).toHaveLength(0)
})

test("repro3: wild trace jump scene routes cleanly with snapshot", () => {
  const circuit = new Circuit()

  circuit.add(
    <board width="10mm" height="8mm" routingDisabled>
      <OneByOnePad name="U1" pcbX={-1} pcbY={0} layer="top" />
      <OneByOnePad name="U2" pcbX={3} pcbY={-1.7} layer="bottom" />
      <chip name="U_obstacle" pcbX={1} pcbY={0}>
        <footprint>
          <smtpad
            pcbX={0}
            pcbY={0}
            shape="rect"
            width="1mm"
            height="2mm"
            layer="top"
            portHints={["pin1"]}
          />
        </footprint>
      </chip>
      <trace from=".U1 > .pin1" to=".U2 > .pin1" />
    </board>,
  )

  const circuitJson = circuit.getCircuitJson()
  const input = getSimpleRouteJson(circuitJson as any, { layerCount: 2 })

  const autorouter = new MultilayerIjump({ input, VIA_COST: 1, debug: true })
  const solution = autorouter.solveAndMapToTraces()

  expect(solution).toHaveLength(1)

  expect(
    convertCircuitJsonToPcbSvg(circuitJson.concat(solution as any) as any),
  ).toMatchSvgSnapshot(import.meta.path)
})
