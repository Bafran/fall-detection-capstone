import { useCallback, useEffect, useMemo, useState } from "react"

export default function useBrowserLocation(options) {
  const geoOptions = useMemo(() => ({
    enableHighAccuracy: true,
    maximumAge: 10_000,
    timeout: 10_000,
    ...options,
  }), [options])

  const [state, setState] = useState({
    status: "idle", // idle | loading | ok | error | unsupported | denied
    coords: null,   // { lat, lng, accuracy }
    timestamp: null,
    error: null,
  })

  const requestOnce = useCallback(() => {
    if (!("geolocation" in navigator)) {
      setState(s => ({ ...s, status: "unsupported", error: "Geolocation not supported" }))
      return
    }

    setState(s => ({ ...s, status: "loading", error: null }))

    navigator.geolocation.getCurrentPosition(
      (pos) => {
        setState({
          status: "ok",
          coords: {
            lat: pos.coords.latitude,
            lng: pos.coords.longitude,
            accuracy: pos.coords.accuracy,
          },
          timestamp: pos.timestamp,
          error: null,
        })
      },
      (err) => {
        let status = "error"
        if (err.code === 1) status = "denied"
        setState(s => ({ ...s, status, error: err.message }))
      },
      geoOptions
    )
  }, [geoOptions])

  // optional: continuous tracking
  useEffect(() => {
    if (!("geolocation" in navigator)) return
    const id = navigator.geolocation.watchPosition(
      (pos) => {
        setState({
          status: "ok",
          coords: {
            lat: pos.coords.latitude,
            lng: pos.coords.longitude,
            accuracy: pos.coords.accuracy,
          },
          timestamp: pos.timestamp,
          error: null,
        })
      },
      () => {},
      geoOptions
    )
    return () => navigator.geolocation.clearWatch(id)
  }, [geoOptions])

  return { ...state, requestOnce }
}