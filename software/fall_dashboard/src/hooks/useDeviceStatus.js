import { useEffect, useState } from "react"

export default function useDeviceStatus() {
  const [status, setStatus] = useState(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState(null)

  useEffect(() => {
    let intervalId

    async function fetchStatus() {
      try {
        const res = await fetch("http://localhost:5050/api/status")
        if (!res.ok) {
          throw new Error(`HTTP ${res.status}`)
        }

        const data = await res.json()
        setStatus(data)
        setError(null)
      } catch (err) {
        setError(err.message)
      } finally {
        setLoading(false)
      }
    }

    fetchStatus()

    // poll every 2 seconds
    intervalId = setInterval(fetchStatus, 2000)

    return () => clearInterval(intervalId)
  }, [])

  return { status, loading, error }
}