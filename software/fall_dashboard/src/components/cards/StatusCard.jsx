import { Heart, Battery, ShieldCheck, MapPin } from "lucide-react"
import "./statusCard.css"

const toneColor = {
  ok: "var(--ok)",
  danger: "var(--danger)",
  neutral: "var(--text)",
}

function Icon({ name, color = "#334155", filled = false }) {
  const common = {
    size: 18,
    stroke: color,
    strokeWidth: 2,
    fill: filled ? color : "none",
  }

  if (name === "heart") return <Heart {...common} />
  if (name === "battery") return <Battery {...common} />
  if (name === "shield") return <ShieldCheck {...common} />
  if (name === "pin") return <MapPin {...common} />
  return null
}

export default function StatusCard({ title, value, sub, tone = "neutral", icon, percent }) {
  const color = toneColor[tone] ?? toneColor.neutral
  const isBattery = icon === "battery" && typeof percent === "number"
  const iconColor =
    icon === "heart" ? "#ef4444" :
    icon === "battery" ? "#14b8a6" :
    icon === "shield" ? (tone === "ok" ? "#22c55e" : tone === "danger" ? "#ef4444" : "#334155") :
    icon === "pin" ? (tone === "ok" ? "#14b8a6" : tone === "danger" ? "#ef4444" : "#334155") :
    "#334155"

  const fill =
    !isBattery ? "rgba(20,184,166,.95)" :
    percent <= 20 ? "rgba(239,68,68,.95)" :
    percent <= 40 ? "rgba(245,158,11,.95)" :
    "rgba(20,184,166,.95)"

  return (
    <div className="sc-card">
    <div className="sc-top">
        <div className="sc-title">{title}</div>

        {icon ? (
            <div className="sc-iconWrap" style={{ background: `${iconColor}1A` }}>
            <Icon name={icon} color={iconColor} filled />
            </div>
        ) : null}

    </div>

      <div className="sc-value" style={{ color }}>
        {value}
      </div>

      {isBattery ? (
        <div className="sc-battery">
          <div className="sc-batteryBar">
            <div
              className="sc-batteryFill"
              style={{
                width: `${Math.max(0, Math.min(100, percent))}%`,
                background: fill,
              }}
            />
          </div>
          <div className="sc-batteryMeta">{percent}%</div>
        </div>
      ) : null}

      <div className="sc-sub">{sub}</div>
    </div>
  )
}