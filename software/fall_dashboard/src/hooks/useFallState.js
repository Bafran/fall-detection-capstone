import { useEffect, useState } from "react"

// Polls the Python server_ui.py backend for latest fall state.
// Expects server_ui.py running on the same machine: http://localhost:5000/api/state
export default function useFallState() {
  const [state, setState] = useState(null)

  useEffect(() => {
    let cancelled = false

    async function tick() {
      try {
        const res = await fetch("http://localhost:5000/api/state")
        if (!res.ok) return
        const json = await res.json()
        if (!cancelled) setState(json)
      } catch {
        // ignore errors; UI can show "Unknown"
      }
    }

    tick()
    const id = setInterval(tick, 1000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [])

  return state
}

