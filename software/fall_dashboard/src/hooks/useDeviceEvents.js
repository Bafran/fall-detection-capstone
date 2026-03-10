import { useEffect, useState } from "react"

export default function useDeviceEvents() {
  const [events, setEvents] = useState([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState(null)

  useEffect(() => {
    let intervalId

    async function fetchEvents() {
      try {
        const res = await fetch("http://localhost:5050/api/events")
        if (!res.ok) {
          throw new Error(`HTTP ${res.status}`)
        }

        const data = await res.json()
        setEvents(Array.isArray(data) ? data : [])
        setError(null)
      } catch (err) {
        setError(err.message)
      } finally {
        setLoading(false)
      }
    }

    fetchEvents()
    intervalId = setInterval(fetchEvents, 2000)

    return () => clearInterval(intervalId)
  }, [])

  return { events, loading, error }
}