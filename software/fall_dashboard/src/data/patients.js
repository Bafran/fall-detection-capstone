/** List of patients for the caregiver dashboard. */
export const PATIENTS = [
  { id: "mario", name: "Mario" },
  { id: "luigi", name: "Luigi" },
]

export function getPatientById(id) {
  return PATIENTS.find((p) => p.id === id) ?? null
}
