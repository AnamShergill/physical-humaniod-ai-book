# Depth and Hierarchy Guidelines

## Overview
This document provides comprehensive guidelines for the depth and hierarchy system implemented in the fresh UI redesign. These guidelines ensure consistent visual design across all pages and components.

## Depth System

### Shadow Levels
- **Level 0**: `none` - Base background elements
- **Level 1**: `0 1px 3px rgba(0, 0, 0, 0.12), 0 1px 2px rgba(0, 0, 0, 0.24)` - Light content containers
- **Level 2**: `0 3px 6px rgba(0, 0, 0, 0.16), 0 3px 6px rgba(0, 0, 0, 0.23)` - Main content blocks
- **Level 3**: `0 10px 20px rgba(0, 0, 0, 0.19), 0 6px 6px rgba(0, 0, 0, 0.23)` - Important cards and featured content
- **Level 4**: `0 14px 28px rgba(0, 0, 0, 0.25), 0 10px 10px rgba(0, 0, 0, 0.22)` - Floating elements and modals
- **Level 5**: `0 19px 38px rgba(0, 0, 0, 0.30), 0 15px 12px rgba(0, 0, 0, 0.22)` - Top-level elements

### Usage Guidelines
- Use Level 1 for standard content containers
- Use Level 2 for important sections that need emphasis
- Use Level 3 for featured content and key components
- Use Level 4 sparingly for floating actions and modals
- Use Level 5 only for critical elements that need maximum attention

## Visual Hierarchy

### Typography Hierarchy
- **H1**: 2.5rem, 700 weight - Main page titles
- **H2**: 2rem, 600 weight - Section headings
- **H3**: 1.5rem, 600 weight - Subsection headings
- **H4**: 1.25rem, 600 weight - Minor headings
- **H5**: 1.1rem, 600 weight - Label headings
- **H6**: 1rem, 600 weight - Supporting headings
- **Body**: 1rem, 400 weight - Main content
- **Small**: 0.875rem, 400 weight - Supporting text

### Color Hierarchy
- **Primary**: For main actions and important elements
- **Secondary**: For supporting actions and elements
- **Accent**: For highlights and special emphasis
- **Success**: For positive feedback
- **Warning**: For cautionary information
- **Danger**: For error states and warnings

## Component Hierarchy

### Cards
- **Standard Cards**: Level 1 depth, subtle shadows
- **Featured Cards**: Level 2 depth, more pronounced shadows
- **Important Cards**: Level 3 depth, heavy shadows with colored accents

### Navigation
- **Sidebar Items**: Hover effects with subtle movement
- **Breadcrumb Items**: Clear visual path with separators
- **Pagination Items**: Directional indicators with depth
- **Tab Navigation**: Active state with depth and color

### Content Blocks
- **Standard Content**: Level 1 depth with subtle borders
- **Highlighted Content**: Gradient backgrounds with accent colors
- **Featured Content**: Level 2 depth with prominent styling
- **Interactive Content**: Hover effects with depth changes

## Responsive Considerations

### Mobile
- Reduce depth effects to improve performance
- Maintain adequate touch targets (44px minimum)
- Simplify complex visual effects
- Preserve hierarchy through spacing and typography

### Tablet
- Moderate depth effects for better visual separation
- Balanced approach between desktop and mobile
- Maintain readability across screen sizes

### Desktop
- Full depth effects for rich visual experience
- Maximum visual hierarchy through shadows and depth
- Detailed interactive states

## Accessibility Guidelines

### Color Contrast
- Maintain WCAG 2.1 AA compliance
- Ensure text is readable against all backgrounds
- Test color combinations for accessibility

### Interactive Elements
- Clear focus states for keyboard navigation
- Sufficient contrast for hover and active states
- Consistent interactive behavior across components

### Visual Clarity
- Avoid excessive visual effects that reduce readability
- Maintain clear visual hierarchy across all elements
- Ensure depth doesn't compromise content legibility

## Implementation Standards

### CSS Classes
- Use semantic class names that reflect depth level
- Maintain consistency in naming conventions
- Document custom properties and their usage

### Component Structure
- Implement depth through CSS custom properties
- Use consistent border-radius values
- Apply shadows through standardized classes

### Performance
- Optimize shadow rendering for performance
- Use hardware acceleration where appropriate
- Minimize repaints and reflows

## Testing Guidelines

### Visual Testing
- Verify depth effects across different browsers
- Test responsive behavior on various screen sizes
- Validate accessibility compliance

### Performance Testing
- Measure impact of depth effects on performance
- Ensure smooth animations and transitions
- Optimize for different device capabilities

## Maintenance

### Updates
- Document changes to depth and hierarchy system
- Maintain backward compatibility where possible
- Update guidelines as system evolves

### Consistency
- Regular audits of visual hierarchy across site
- Consistent application of depth effects
- Alignment with overall design system