# AI Usage Disclaimer for the embedded programming course project

## Overview

Our 4-member group used Generative AI tools (ChatGPT, Claude) as a supplementary resource during development of the project.

## How AI assisted us

### Debugging

- Helped identify functionality issues in code
- Assisted with debugging cross-member contribution conflicts
- Suggested performance and correctness improvements
- Finding typos and incorrect values

### Code integration

- Merged individually developed subsystems into a cohesive codebase
- Resolved conflicts between different team members' implementations
- Unified coding style and naming conventions

### Documentation

- Improved inline comments for readability
- Cleaned up leftover debug code and artifacts

## Our approach

All AI-generated suggestions were reviewed, tested, and understood by team members before integration. The core algorithms, hardware design, and original source code were developed by the team members.

## A few examples of the prompts used

Below are representative prompts used during the project. They align with LUT allowed AI usage policy (structuring information, ideation/drafting, concept clarification, and language refinement):

**Debugging assistance:**

> "In modulation mode the LED blinks instead of smoothly changing brightness. What should we check first in our PWM/timer/control-loop path?"

> "Review our control-loop logic and point out likely reasons PWM duty is not updating as intended."

> "Does this PI controller step function look correct, especially the anti-windup back-calculation part?"

> "Could an incorrect measurement update explain the observed behavior? What variables should we log to confirm?"

**Understanding concepts:**

> "What does x_state represent in this model, and how can we verify it updates correctly at runtime?"

> "What is required to enable and configure a GPIO pin on STM32 (clock enable, mode, alternate function if needed)?"

**Code integration:**

> "We have separate working modules (UART commands, EXTI button, PWM output, control update). What is a clean non-blocking way to integrate them into main.c with deterministic timing?"

**Documentation:**

> "Rewrite these inline comments so the register bit operations and intent are clearer, without changing the code."
