export function haversineMeters(a, b) {
  // a, b: { lat, lng }
  const R = 6371000
  const toRad = (deg) => (deg * Math.PI) / 180

  const dLat = toRad(b.lat - a.lat)
  const dLng = toRad(b.lng - a.lng)

  const lat1 = toRad(a.lat)
  const lat2 = toRad(b.lat)

  const sinDLat = Math.sin(dLat / 2)
  const sinDLng = Math.sin(dLng / 2)

  const h =
    sinDLat * sinDLat +
    Math.cos(lat1) * Math.cos(lat2) * sinDLng * sinDLng

  return 2 * R * Math.asin(Math.sqrt(h))
}

/**
 * Geocode an address to lat/lng using OpenStreetMap Nominatim.
 * Use sparingly; Nominatim requires a descriptive User-Agent.
 * @param {string} query - Address or place name
 * @returns {Promise<{ lat: number, lng: number, displayName: string } | null>}
 */
export async function geocodeAddress(query) {
  const trimmed = query?.trim()
  if (!trimmed) return null
  const params = new URLSearchParams({
    q: trimmed,
    format: "json",
    limit: "1",
  })
  const res = await fetch(
    `https://nominatim.openstreetmap.org/search?${params}`,
    {
      headers: {
        Accept: "application/json",
        "Accept-Language": "en",
        "User-Agent": "FallDetectionDashboard/1.0 (caregiver home zone setup)",
      },
    }
  )
  if (!res.ok) return null
  const data = await res.json()
  const first = data?.[0]
  if (!first?.lat || !first?.lon) return null
  return {
    lat: Number(first.lat),
    lng: Number(first.lon),
    displayName: first.display_name ?? trimmed,
  }
}