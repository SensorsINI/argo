Sealing moving parts against water—especially under low-tension or small-scale conditions—requires balancing the water tightness of the seal against the **friction** it introduces. Because you are working with small diameters (1.3mm and 0.5mm) and limited tension, traditional high-pressure industrial seals will cause too much binding.

Given your access to **PETG** (for rigid structures) and **TPU** (for flexible seals), here are the best methods to handle both of your specific scenarios.

---

## 1. Sealing the 1.3mm Sliding Rod

A sliding rod (axial movement) is best sealed using a **bellows** or a **tight-tolerance elastomeric pass-through**. Since a 1.3mm rod has very little surface area, a mechanical lip seal might bind it completely.

### Method A: The TPU Sealing Bellows (Highly Recommended)

Instead of sealing *against* the moving rod, you seal the rod permanently to a flexible boot. The boot flexes as the rod moves back and forth, resulting in **zero sliding friction** and a 100% waterproof seal.

* **How it works:** You 3D print a thin-walled, accordion-style bellows out of TPU. One end is tightly clamped or glued to the 1.3mm rod, and the other end is clamped to a rigid PETG through-hull fitting (printed to fit your 6mm opening).
* **Pros:** Absolutely watertight; zero friction on the rod.
* **Cons:** Limits the total travel distance to the expansion/contraction limit of the TPU bellows.

### Method B: Lubricated Pushrod Guide Tube

Commonly used in RC marine applications, this relies on a close-tolerance tube packed with viscous, water-insoluble grease.

* **How it works:** Print a PETG sleeve that slips into your 6mm hull opening. The inside diameter (ID) of this sleeve should be just slightly larger than the rod (e.g., 1.5mm to 1.6mm). Pack the inside of this tube with **marine grease** or **silicone grease**.
* **Pros:** Very low friction; simple to implement; unlimited travel distance.
* **Cons:** The grease will slowly migrate out over time and requires periodic reapplication.

---

## 2. Sealing the 0.5mm Dyneema Winch Line

Sealing a braided or monofilament line is notoriously difficult because water can wick through the braid, and the line lacks the rigidity to push past a tight rubber seal without bunching up—especially with an elastic return cord.

### Method A: The High-Tube / Snorkel System (Highly Recommended)

If the winch line is exiting the hull *above* the waterline (or if the hull only experiences brief splashes/heeling rather than total submersion), the best seal is a **gravity-based snorkel tube**.

* **How it works:** Print a rigid PETG tube that mounts into your 6mm hull opening. Route this tube internally so that it extends **well above the external waterline** of the vessel before the line exits the tube into the hull interior.
* **Pros:** **Zero friction** on your 0.5mm line, allowing the elastic cord to work perfectly. 100% reliable as long as the top of the tube remains higher than the water level outside.
* **Cons:** Does not protect against complete submersion or capsizing.

### Method B: TPU Squeegee / Grommet with Marine Grease

If the opening *must* be submerged, you need a physical barrier, but it must be incredibly compliant.

* **How it works:** Print a PETG plug for the 6mm hole with a hollow internal chamber. Caps on either side of the chamber should have a tiny 0.6mm hole. Print a thin TPU diaphragm (0.4mm to 0.8mm thick) with a tiny pinhole (approx. 0.3mm) for the line to pass through. Fill the internal PETG chamber with heavy silicone grease.
* **Pros:** Works dynamically under water.
* **Cons:** Dyneema is inherently slippery but textured; it will drag tiny amounts of water past the seal via capillary action, and the friction might require you to beef up your elastic return cord.

---

## Practical 3D Printing & Design Tips

* **Perimeter Settings for PETG:** When printing your through-hull fittings or tubes from PETG, increase your perimeter/wall count so that the part is entirely solid walls rather than infill. Print slightly hot to ensure maximum layer adhesion, preventing water from weeping *through* the 3D printed layers themselves.
* **TPU Thickness:** To keep friction low on a 0.5mm line or a 1.3mm rod, your TPU seals need to be incredibly thin. Design them to be only **1 to 2 perimeters thick** (using a 0.4mm nozzle) so they remain highly flexible.
* **Coating the Line:** To prevent the Dyneema line from acting like a wick, rub it thoroughly with beeswax or silicone grease before threading it through the seal. This fills the microscopic voids in the braid.