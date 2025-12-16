import React, { useState } from 'react';

export default function QuizBlock({ question, options, correctAnswer, explanation }) {
  const [selectedAnswer, setSelectedAnswer] = useState(null);
  const [showResult, setShowResult] = useState(false);

  const handleAnswer = (option) => {
    setSelectedAnswer(option);
    setShowResult(true);
  };

  return (
    <div className="bg-white border border-gray-200 rounded-lg p-6 my-6 shadow-sm hover:shadow-md transition-shadow duration-200">
      <h4 className="font-semibold text-gray-900 mb-4 flex items-center gap-2">
        <span className="bg-blue-100 text-blue-800 px-2 py-1 rounded-full text-sm">Quiz</span>
        {question}
      </h4>

      <div className="space-y-2">
        {options.map((option, index) => (
          <button
            key={index}
            onClick={() => handleAnswer(option)}
            disabled={showResult}
            className={`w-full text-left p-3 rounded-lg border transition-all duration-200 ${
              selectedAnswer === option
                ? option === correctAnswer
                  ? 'border-green-500 bg-green-50 text-green-800'
                  : 'border-red-500 bg-red-50 text-red-800'
                : 'border-gray-200 hover:border-blue-300 hover:bg-blue-50'
            } ${showResult ? 'cursor-default' : 'cursor-pointer'}`}
          >
            <span className="font-medium">{String.fromCharCode(65 + index)}.</span> {option}
          </button>
        ))}
      </div>

      {showResult && (
        <div className={`mt-4 p-4 rounded-lg ${
          selectedAnswer === correctAnswer
            ? 'bg-green-50 border border-green-200'
            : 'bg-red-50 border border-red-200'
        }`}>
          <p className={`font-medium ${selectedAnswer === correctAnswer ? 'text-green-800' : 'text-red-800'}`}>
            {selectedAnswer === correctAnswer ? '✅ Correct!' : '❌ Incorrect'}
          </p>
          {explanation && <p className="mt-2 text-gray-700">{explanation}</p>}
        </div>
      )}
    </div>
  );
}