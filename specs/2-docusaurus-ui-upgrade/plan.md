# Implementation Plan: Docusaurus UI Upgrade

**Branch**: `2-docusaurus-ui-upgrade` | **Date**: 2026-01-01 | **Spec**: [link to spec.md](../2-docusaurus-ui-upgrade/spec.md)
**Input**: Feature specification from `/specs/2-docusaurus-ui-upgrade/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a comprehensive UI upgrade for the existing Docusaurus-based AI-Native Textbook. This upgrade will transform the interface into a clean, modern, professional, book-like experience optimized for long-form reading. The solution maintains all existing functionality while enhancing typography, spacing, navigation, visual hierarchy, and chatbot integration. The approach uses Docusaurus theming capabilities with custom CSS and React components to achieve the desired aesthetic and user experience improvements.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+), CSS with custom properties
**Primary Dependencies**: Docusaurus, existing React components, CSS
**Storage**: CSS files for styling, React components for UI elements
**Testing**: Visual regression tests, manual UI testing, responsive testing
**Target Platform**: Web-based (React/Docusaurus), compatible with modern browsers
**Performance Goals**: Maintain current content loading speeds (<3 seconds), optimize for low-end devices
**Constraints**: Must not break existing functionality, maintain accessibility standards, follow Docusaurus conventions
**Scale/Scope**: Affects homepage and all chapter/lesson pages, impacts all users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Academic + Modern Aesthetic**: ✅ Will implement clean, professional design without flashy elements
- **Minimal, Readable, Distraction-Free**: ✅ Focus on typography and spacing for long-form reading
- **Optimized for Long-Form Reading**: ✅ Comfortable line width, increased line height, clear hierarchy
- **Mobile-First Responsive**: ✅ Design will work across all device sizes
- **Consistent Spacing and Typography**: ✅ Implement systematic approach to design elements
- **Non-Intrusive Chatbot**: ✅ Chatbot will be visible but not distracting
- **Visual Hierarchy (Chapters → Lessons)**: ✅ Clear structure showing relationships
- **Performance Maintenance**: ✅ No significant performance impact on low-end devices

## Project Structure

### Documentation (this feature)

```text
specs/2-docusaurus-ui-upgrade/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── ui-upgrade-api.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code Changes

```text
src/
├── pages/
│   └── index.js         # Updated homepage with new design
├── theme/
│   ├── Root.js          # Global theme customizations for typography/spacing
│   ├── MDXComponents.js # Enhanced MDX components for content display
│   └── Layout.js        # Custom layout with improved navigation
├── components/
│   ├── Homepage/        # New homepage components
│   │   ├── HeroSection/
│   │   ├── FeaturesSection/
│   │   └── CTASection/
│   ├── Layout/
│   │   ├── Navigation/
│   │   ├── Sidebar/
│   │   └── Breadcrumb/
│   ├── Reading/
│   │   ├── Typography/
│   │   ├── LessonHeader/
│   │   └── SectionDivider/
│   └── Chatbot/
│       └── FloatingAssistant/  # Non-intrusive chatbot UI
├── css/
│   ├── custom.css       # Custom CSS overrides for typography/spacing
│   └── responsive.css   # Mobile-first responsive styles
└── styles/
    └── theme.css        # CSS custom properties for consistent theming
```

### Configuration Updates

```text
docusaurus.config.js     # Updated theme configuration for new UI
sidebars.js              # Potentially updated navigation structure
package.json             # Potentially new dev dependencies for styling
```

**Structure Decision**: Leverage Docusaurus theming system with custom components and CSS to achieve the desired UI upgrade. This approach maintains compatibility with existing content while allowing for comprehensive design improvements. The architecture follows Docusaurus best practices while implementing the required academic, modern aesthetic with proper visual hierarchy.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Custom Components | Required for non-intrusive chatbot placement | Simple CSS changes would not allow proper chatbot integration |
| Layout Modifications | Needed for improved visual hierarchy | Pure styling would not address navigation structure needs |
| Homepage Rewrite | Required for academic aesthetic goals | Minor styling changes would not achieve desired homepage impact |