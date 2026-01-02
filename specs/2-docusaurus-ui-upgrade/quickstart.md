# Quickstart Guide: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade for AI-Native Textbook (Feature #2)
**Created**: 2026-01-01
**Last Updated**: 2026-01-01

## Overview

This quickstart guide provides the essential information to begin implementing the Docusaurus UI upgrade for the AI-Native Textbook. The upgrade focuses on creating a clean, modern, professional, book-like interface that enhances the reading experience.

## Prerequisites

Before starting the UI upgrade implementation, ensure you have:

- Node.js 18+ installed
- The existing AI-Native Textbook project set up and running
- Docusaurus development environment configured
- Access to the existing content structure (chapters/lessons)
- Understanding of Docusaurus theming system

## Setup Commands

1. **Clone/ensure the project is available:**
```bash
# Navigate to the project directory
cd C:\Users\Bruno\Desktop\AI-BOOK
```

2. **Install dependencies (if not already done):**
```bash
npm install
```

3. **Start development server:**
```bash
npm start
```

## Implementation Quick Start

### 1. Start with Foundation (Required)
```bash
# Begin with creating the foundational files
# - Create CSS custom properties for theming
# - Set up responsive CSS framework
# - Configure theme components
```

### 2. Homepage Redesign (Priority 1)
Start with the homepage redesign as it provides immediate visual impact:

```bash
# Create homepage components:
# - HeroSection with book title/subtitle
# - FeaturesSection highlighting textbook features
# - CTASection with primary/secondary CTAs
# - Update src/pages/index.js with new components
```

### 3. Development Workflow
```bash
# During development, test changes with:
npm start

# To build and test the production version:
npm run build
npm run serve
```

## Key Files to Modify

### Homepage
- `src/pages/index.js` - Main homepage
- `src/components/Homepage/` - New homepage components

### Theme & Styling
- `src/theme/` - Custom theme components
- `src/css/custom.css` - Custom CSS overrides
- `src/styles/theme.css` - CSS custom properties

### Layout & Components
- `docusaurus.config.js` - Theme configuration updates
- `src/components/Layout/` - Layout components
- `src/components/Reading/` - Reading experience components
- `src/components/Chatbot/` - Non-intrusive chatbot UI

## Testing Your Changes

### Local Development
```bash
# Start development server
npm start

# Visit http://localhost:3000 to see your changes
```

### Production Build Test
```bash
# Build the project
npm run build

# Serve the built version locally
npm run serve
```

## Common Tasks

### 1. Adding Typography System
```css
/* Define in src/styles/theme.css */
:root {
  --ifm-font-size-base: 16px;
  --text-font-family: 'Inter', system-ui, sans-serif;
  --heading-font-family: 'Inter', system-ui, sans-serif;
  --text-line-height: 1.7;
  --text-line-width: 65ch; /* Optimal for reading */
}
```

### 2. Creating Responsive Layout
```css
/* In src/css/custom.css */
@media (min-width: 997px) {
  .container {
    max-width: 800px; /* Optimal reading width */
  }
}
```

### 3. Component Structure
```
src/
├── components/
│   ├── Homepage/
│   │   ├── HeroSection/
│   │   ├── FeaturesSection/
│   │   └── CTASection/
│   ├── Layout/
│   ├── Reading/
│   └── Chatbot/
```

## Validation Checklist

- [ ] Homepage has clean, professional academic aesthetic
- [ ] Typography is optimized for long-form reading (65-75 characters line width)
- [ ] Line height is comfortable (1.6-1.8 ratio)
- [ ] Navigation shows clear visual hierarchy (chapters → lessons)
- [ ] Chatbot UI is visible but not distracting
- [ ] Design is responsive across mobile, tablet, desktop
- [ ] Performance is maintained (pages load in <3 seconds)
- [ ] Accessibility standards are met (contrast ratios, keyboard nav)

## Next Steps

1. Complete the foundational setup (Phase 1-2 in tasks.md)
2. Implement homepage redesign (Phase 3)
3. Enhance reading experience (Phase 4)
4. Improve navigation and layout (Phase 5)
5. Complete typography and spacing (Phase 6)
6. Integrate chatbot UI (Phase 7)
7. Implement responsive design (Phase 8)
8. Test and validate (Phase 9)

## Troubleshooting

### If changes don't appear:
- Clear Docusaurus cache: `npx docusaurus clear`
- Restart development server: `npm start`

### If build fails:
- Check for CSS syntax errors
- Verify component imports/exports
- Review Docusaurus configuration

### Performance issues:
- Minimize CSS bundle size
- Optimize images and assets
- Use efficient selectors