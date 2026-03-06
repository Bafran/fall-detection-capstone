import { BrowserRouter, Routes, Route, Navigate } from "react-router-dom"
import PatientsList from "./pages/PatientsList.jsx"
import Dashboard from "./pages/Dashboard.jsx"

export default function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Navigate to="/patients" replace />} />
        <Route path="/patients" element={<PatientsList />} />
        <Route path="/patients/:patientId" element={<Dashboard />} />
        <Route path="*" element={<Navigate to="/patients" replace />} />
      </Routes>
    </BrowserRouter>
  )
}
