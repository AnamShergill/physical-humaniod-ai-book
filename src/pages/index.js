import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styled from 'styled-components';

const HeroSection = styled.section`
  background: linear-gradient(135deg, #0f4c75 0%, #3282b8 100%); /* Deep blue to teal - tech/robotics theme */
  color: white;
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;

  &::before {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background: radial-gradient(circle at top right, rgba(255,255,255,0.1) 0%, transparent 50%);
    pointer-events: none;
  }
`;


const HeroContainer = styled.div`
  position: relative;
  z-index: 2;
  max-width: 800px;
  margin: 0 auto;
  padding: 0 1rem;
`;

const HeroTitle = styled.h1`
  font-size: 3.5rem;
  font-weight: 700;
  margin-bottom: 1rem;
  animation: fadeInUp 1s ease-out;

  @media (max-width: 768px) {
    font-size: 2.5rem;
  }
`;

const HeroSubtitle = styled.p`
  font-size: 1.5rem;
  margin-bottom: 2rem;
  opacity: 0.9;
  animation: fadeInUp 1s ease-out 0.2s both;

  @media (max-width: 768px) {
    font-size: 1.25rem;
  }
`;

const CTAButton = styled(Link)`
  display: inline-block;
  background: white;
  color: #0f4c75; /* Deep blue to match hero section */
  padding: 1rem 2rem;
  border-radius: 50px;
  text-decoration: none;
  font-weight: 600;
  font-size: 1.1rem;
  transition: all 0.3s ease;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
  animation: fadeInUp 1s ease-out 0.4s both;

  &:hover {
    transform: translateY(-3px);
    box-shadow: 0 15px 40px rgba(0, 0, 0, 0.3);
    background: #f0f8ff; /* Light blue tint */
    text-decoration: none;
  }
`;

const FeaturesSection = styled.section`
  padding: 5rem 0;
  background: #f0f7ff; /* Light blue background to complement the theme */
`;

const FeaturesContainer = styled.div`
  max-width: 1200px;
  margin: 0 auto;
  padding: 0 1rem;
`;

const SectionTitle = styled.h2`
  text-align: center;
  font-size: 2.5rem;
  font-weight: 700;
  margin-bottom: 3rem;
  color: #1f2937;

  @media (max-width: 768px) {
    font-size: 2rem;
  }
`;

const FeaturesGrid = styled.div`
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  margin-top: 3rem;
`;

const FeatureCard = styled.div`
  background: white;
  padding: 2rem;
  border-radius: 1rem;
  box-shadow: 0 10px 30px rgba(15, 76, 117, 0.1); /* Blue-tinged shadow */
  transition: all 0.3s ease;
  text-align: center;

  &:hover {
    transform: translateY(-10px);
    box-shadow: 0 20px 40px rgba(15, 76, 117, 0.2);
  }
`;

const FeatureIcon = styled.div`
  width: 80px;
  height: 80px;
  margin: 0 auto 1.5rem;
  background: linear-gradient(135deg, #0f4c75 0%, #3282b8 100%); /* Tech blue theme */
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 2rem;
  color: white;
`;


const FeatureTitle = styled.h3`
  font-size: 1.5rem;
  font-weight: 600;
  margin-bottom: 1rem;
  color: #1f2937;
`;

const FeatureDescription = styled.p`
  color: #6b7280;
  line-height: 1.6;
`;

const StatsSection = styled.section`
  padding: 5rem 0;
  background: linear-gradient(135deg, #0f4c75 0%, #3282b8 100%); /* Blue theme you requested */
  color: white;
  text-align: center;
`;

const StatsContainer = styled.div`
  max-width: 1200px;
  margin: 0 auto;
  padding: 0 1rem;
`;

const StatsGrid = styled.div`
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 3rem;
  margin-top: 3rem;
`;

const StatItem = styled.div`
  animation: fadeInUp 1s ease-out;
`;

const StatNumber = styled.div`
  font-size: 3rem;
  font-weight: 700;
  margin-bottom: 0.5rem;
`;

const StatLabel = styled.div`
  font-size: 1.1rem;
  opacity: 0.9;
`;

// Add keyframes for animations
const GlobalStyles = styled.div`
  @keyframes fadeInUp {
    from {
      opacity: 0;
      transform: translateY(30px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }

  @keyframes float {
    0%, 100% {
      transform: translateY(0px);
    }
    50% {
      transform: translateY(-20px);
    }
  }
`;

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  const features = [
    {
      title: 'Physical AI Fundamentals',
      description: 'Learn the core principles of AI systems that interact with the physical world, including embodied intelligence and physics-informed learning.',
      icon: '🧠'
    },
    {
      title: 'Humanoid Robotics',
      description: 'Explore the design, control, and intelligence systems that power next-generation humanoid robots and their applications.',
      icon: '🤖'
    },
    {
      title: 'Practical Applications',
      description: 'Real-world examples and hands-on exercises to help you implement Physical AI and robotics concepts.',
      icon: '🛠️'
    }
  ];

  const stats = [
    { number: '8+', label: 'Chapters' },
    { number: '24+', label: 'Lessons' },
    { number: '100+', label: 'Code Examples' },
    { number: '∞', label: 'Possibilities' }
  ];

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-native textbook covering Physical AI & Humanoid Robotics">
      <GlobalStyles>
        <HeroSection>
          <HeroContainer>
            <HeroTitle>{siteConfig.title}</HeroTitle>
            <HeroSubtitle>{siteConfig.tagline}</HeroSubtitle>
            <CTAButton to="/docs/intro">
              Start Learning →
            </CTAButton>
          </HeroContainer>
        </HeroSection>

        <FeaturesSection>
          <FeaturesContainer>
            <SectionTitle>What You'll Learn</SectionTitle>
            <FeaturesGrid>
              {features.map((feature, index) => (
                <FeatureCard key={index}>
                  <FeatureIcon>{feature.icon}</FeatureIcon>
                  <FeatureTitle>{feature.title}</FeatureTitle>
                  <FeatureDescription>{feature.description}</FeatureDescription>
                </FeatureCard>
              ))}
            </FeaturesGrid>
          </FeaturesContainer>
        </FeaturesSection>

        <StatsSection>
          <StatsContainer>
            <SectionTitle>Learning Journey</SectionTitle>
            <StatsGrid>
              {stats.map((stat, index) => (
                <StatItem key={index}>
                  <StatNumber>{stat.number}</StatNumber>
                  <StatLabel>{stat.label}</StatLabel>
                </StatItem>
              ))}
            </StatsGrid>
          </StatsContainer>
        </StatsSection>
      </GlobalStyles>
    </Layout>
  );
}
